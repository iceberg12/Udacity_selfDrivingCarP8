/*
 * particle_filter.cpp
 *
 *  Created on: Aug 12, 2017
 *      Author: Minh Nguyen
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

static default_random_engine gen;

const int init_num_particles = 50;
const double init_weight = 1.0;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	num_particles = init_num_particles;
	
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
	
	for (int i=0; i<num_particles; i++){
		Particle p = {i, dist_x(gen), dist_y(gen), dist_theta(gen), init_weight};
		particles.push_back(p);
		weights.push_back(init_weight);
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);
	
	for (unsigned i = 0; i < num_particles; i++) {

    	// calculate new state
    	if (fabs(yaw_rate) < 0.00001) {  
      		particles[i].x += velocity * delta_t * cos(particles[i].theta);
      		particles[i].y += velocity * delta_t * sin(particles[i].theta);
    	} 
    	else {
      		particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
      		particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
      		particles[i].theta += yaw_rate * delta_t;
    	}

    	// add noise
    	particles[i].x += dist_x(gen);
    	particles[i].y += dist_y(gen);
    	particles[i].theta += dist_theta(gen);
  	}
  	
  	
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	for (unsigned int i = 0; i < observations.size(); i++) {
		LandmarkObs obs = observations[i];
		double min_dist = numeric_limits<double>::max();
		
		int map_id = -1;
		
		for (unsigned int j = 0; j < predicted.size(); j++) {
      		// grab current prediction
      		LandmarkObs pred = predicted[j];
      
      		// get distance between current/predicted landmarks
      		double cur_dist = dist(obs.x, obs.y, pred.x, pred.y);

      		// find the predicted landmark nearest the current observed landmark
      		if (cur_dist < min_dist) {
        		min_dist = cur_dist;
        		map_id = pred.id;
      		}
      	}
      	
      	observations[i].id = map_id;
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a multi-variate Gaussian distribution.
	// NOTE: The observations are given in the VEHICLE'S coordinate system. The particles are located
	//   according to the MAP'S coordinate system. Hence transform between the two systems needs to be done.
	for (unsigned i=0; i<num_particles; i++){
	
		double p_x = particles[i].x;
		double p_y = particles[i].y;
		double p_theta = particles[i].theta;
		
		// 1. transform observations from vehicle to map coordinates assuming it's the particle observing
		vector<LandmarkObs> observations_on_map;
		
		for (unsigned j=0; j<observations.size(); j++){
			int o_id = observations[j].id;
			double o_x = observations[j].x;
			double o_y = observations[j].y;
		
			double x_new = p_x + o_x*cos(p_theta) - o_y*sin(p_theta);
			double y_new = p_y + o_x*sin(p_theta) + o_y*cos(p_theta);
			
			LandmarkObs o_new = {o_id, x_new, y_new};
			observations_on_map.push_back(o_new);		
		}
		// 2. find landmarks within the particle's range
		vector<LandmarkObs> landmarks_in_range;;
		for (unsigned j=0; j<map_landmarks.landmark_list.size(); j++){
			int lm_id = map_landmarks.landmark_list[j].id_i;
			double lm_x = map_landmarks.landmark_list[j].x_f;
			double lm_y = map_landmarks.landmark_list[j].y_f;
			
			if (dist(p_x, p_y, lm_x, lm_y) < sensor_range){
				landmarks_in_range.push_back(LandmarkObs{lm_id, lm_x, lm_y});
			}	
		}
		
		// 3. find which landmark is likely being observed based on `nearest neighbor` method
		dataAssociation(landmarks_in_range, observations_on_map);
		
		// 4. determine the weights based on the difference between particle observation and actual observation
		particles[i].weight = 1.0;
		
		double std_x = std_landmark[0];
		double std_y = std_landmark[1];
		double na = 2.0 * std_x * std_x;
		double nb = 2.0 * std_y * std_y;
		double gauss_norm = 2.0 * M_PI * std_x * std_y;
		
		for (unsigned j=0; j<observations_on_map.size(); j++){
			int o_id = observations_on_map[j].id;
			double o_x = observations_on_map[j].x;
			double o_y = observations_on_map[j].y;
			
			double pr_x, pr_y;
			for (unsigned int k = 0; k < landmarks_in_range.size(); k++) {
        		if (landmarks_in_range[k].id == o_id) {
          			pr_x = landmarks_in_range[k].x;
          			pr_y = landmarks_in_range[k].y;
          			break;
        		}
      		}
      		double obs_w = 1/gauss_norm * exp( - (pow(pr_x-o_x,2)/na + (pow(pr_y-o_y,2)/nb)) );

      		// product of this obersvation weight with total observations weight
      		particles[i].weight *= obs_w;
		}
		
		weights[i] = particles[i].weight;
	}	
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 

	vector<Particle> new_particles;

  	// get all of the current weights
  	vector<double> weights;
  	for (int i = 0; i < num_particles; i++) {
    	weights.push_back(particles[i].weight);
  	}
  	
  	// Method 1: discrete_distribution*******************************
  	discrete_distribution<int> index(weights.begin(), weights.end());
  	for (unsigned j=0; j<num_particles;j++){
  		const int i = index(gen);
  		new_particles.push_back(particles[i]); 		
  	}
  	
  	// Method 2: Systematic sampling*********************************
  	// Step 1: Get the first 
  	//uniform_int_distribution<int> uniintdist(0, num_particles-1);
  	
  	particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
