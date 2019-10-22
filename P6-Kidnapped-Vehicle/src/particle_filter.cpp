/**
* particle_filter.cpp
*
* Created on: Dec 12, 2016
* Author: Tiffany Huang
*/

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include <cmath>
#include <iterator>
#include "helper_functions.h"

using namespace std;

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	/**
	* TODO: Set the number of particles. Initialize all particles to
	*   first position (based on estimates of x, y, theta and their uncertainties
	*   from GPS) and all weights to 1.
	* TODO: Add random Gaussian noise to each particle.
	* NOTE: Consult particle_filter.h for more information about this method
	*   (and others in this file).
	*/
	num_particles = 42;  // TODO: Set the number of particles
	std::default_random_engine gen;

	std::normal_distribution<double> x_gauss(x, std[0]);
	std::normal_distribution<double> y_gauss(y, std[1]);
	std::normal_distribution<double> theta_gauss(theta, std[2]);




	for (int i = 0; i < num_particles; ++i)
	{
		Particle current_particle;
		current_particle.id = i;
		current_particle.x = x_gauss(gen);
		current_particle.y = y_gauss(gen);
		current_particle.theta = theta_gauss(gen);
		current_particle.weight = 1.0;


		particles.push_back(current_particle);
		weights.push_back(current_particle.weight);
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
	double velocity, double yaw_rate) {
	/**
	* TODO: Add measurements to each particle and add random Gaussian noise.
	* NOTE: When adding noise you may find std::normal_distribution
	*   and std::default_random_engine useful.
	*  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	*  http://www.cplusplus.com/reference/random/default_random_engine/
	*/
	std::default_random_engine gen;



	double new_pos_x, new_pos_y, new_pos_theta;


	for (int i = 0; i < num_particles; ++i)
	{
		double particle_x = particles[i].x;
		double particle_y = particles[i].y;
		double particle_theta = particles[i].theta;

		if (fabs(yaw_rate) < 0.0001)
		{
			new_pos_x = particle_x + velocity * cos(particle_theta) * delta_t;
			new_pos_y = particle_y + velocity * sin(particle_theta) * delta_t;
			new_pos_theta = particle_theta;
		}
		else
		{
			new_pos_x = particle_x + (velocity / yaw_rate) * (sin(particle_theta + (yaw_rate * delta_t)) - sin(particle_theta));
			new_pos_y = particle_y + (velocity / yaw_rate) * (cos(particle_theta) - cos(particle_theta + (yaw_rate * delta_t)));
			new_pos_theta = particle_theta + (yaw_rate * delta_t);
		}


		std::normal_distribution<double> dist_x(new_pos_x, std_pos[0]);
		std::normal_distribution<double> dist_y(new_pos_y, std_pos[1]);
		std::normal_distribution<double> dist_th(new_pos_theta, std_pos[2]);



		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_th(gen);

	}

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
	vector<LandmarkObs>& observations, double sensor_range) {
	/**
	* TODO: Find the predicted measurement that is closest to each
	*   observed measurement and assign the observed measurement to this
	*   particular landmark.
	* NOTE: this method will NOT be called by the grading code. But you will
	*   probably find it useful to implement this method and use it as a helper
	*   during the updateWeights phase.
	*/

	



	for (int i = 0; i < observations.size(); ++i)
	{
		double shortest_dist = sensor_range * sqrt(2);
		int closest_landmark_id = -1;
		double obs_x = observations[i].x;
		double obs_y = observations[i].y;

		for (int j = 0; j < predicted.size(); ++j)
		{
			double pred_x = predicted[j].x;
			double pred_y = predicted[j].y;
			int pred_id = predicted[j].id;

			double current_dist = dist(obs_x, obs_y, pred_x, pred_y);


			if (current_dist <shortest_dist)
			{
				shortest_dist = current_dist;
				observations[i].id = pred_id;

			}

		}
	}




}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
	const vector<LandmarkObs> &observations,
	const Map &map_landmarks) {
	/**
	* TODO: Update the weights of each particle using a mult-variate Gaussian
	*   distribution. You can read more about this distribution here:
	*   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	* NOTE: The observations are given in the VEHICLE'S coordinate system.
	*   Your particles are located according to the MAP'S coordinate system.
	*   You will need to transform between the two systems. Keep in mind that
	*   this transformation requires both rotation AND translation (but no scaling).
	*   The following is a good resource for the theory:
	*   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	*   and the following is a good resource for the actual equation to implement
	*   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
	*/


	double weight_normalizer = 0.0;



	for (int i = 0; i < num_particles; ++i)
	{
		vector <LandmarkObs> map_observations;
		//transform observations from vechile to map coordinates
		double particle_x = particles[i].x;
		double particle_y = particles[i].y;
		double particle_theta = particles[i].theta;

		for (int j = 0; j < observations.size(); ++j)
		{
			double xcar_obs = observations[j].x;
			double ycar_obs = observations[j].y;
			LandmarkObs curr_map_obs;


			curr_map_obs.id = j;
			curr_map_obs.x = particle_x + (cos(particle_theta) * observations[j].x) - (sin(particle_theta) * observations[j].y);
			curr_map_obs.y = particle_y + (sin(particle_theta) * observations[j].x) + (cos(particle_theta) * observations[j].y);


			map_observations.push_back(curr_map_obs);

		}

		//filter map landmars to keep only those within in sensor range
		vector <LandmarkObs> predicted_landmarks;

		for (int k = 0; k < map_landmarks.landmark_list.size(); ++k)
		{
			Map::single_landmark_s current_landmark = map_landmarks.landmark_list[k];

			if ((fabs(particle_x - current_landmark.x_f) <= sensor_range) && (fabs(particle_y - current_landmark.y_f) <= sensor_range))
			{
				predicted_landmarks.push_back(LandmarkObs{ current_landmark.id_i, current_landmark.x_f, current_landmark.y_f });
			}
		}

		//associate observations with predicted landmarks
		dataAssociation(predicted_landmarks, map_observations, sensor_range);

		particles[i].weight = 1.0;

		//calculate weight of particles
		double gauss_norm;
		double sigma_x = std_landmark[0];
		double sigma_y = std_landmark[1];


		double sigma_x2 = sigma_x * sigma_x;
		double sigma_y2 = sigma_y * sigma_y;

		gauss_norm = 1.0 / (2.0 * M_PI*sigma_x*sigma_y);

		for (int l = 0; l < map_observations.size(); ++l)
		{
			double x_obs = map_observations[l].x;
			double y_obs = map_observations[l].y;
			double obs_id = map_observations[l].id;
			double multi_prob = 1.0;

			for (int m = 0; m < predicted_landmarks.size(); ++m)
			{
				double landmark_x = predicted_landmarks[m].x;
				double landmark_y = predicted_landmarks[m].y;
				double landmark_id = predicted_landmarks[m].id;



				if (landmark_id == obs_id)
				{
					multi_prob = gauss_norm * exp(-1.0 * ((pow((x_obs - landmark_x), 2) / (2.0 * sigma_x2)) + (pow((y_obs - landmark_y), 2) / (2.0 * sigma_y2))));

					particles[i].weight *= multi_prob;
				}
			}

		}

		weight_normalizer += particles[i].weight;
	}



	//normalize weights because reasample use probalistic approch
	for (int i = 0; i < num_particles; ++i)
	{
		double weight = particles[i].weight / weight_normalizer;
		particles[i].weight = weight;
		weights[i] = weight;


	}



}

void ParticleFilter::resample() {
	/**
	* TODO: Resample particles with replacement with probability proportional
	*   to their weight.
	* NOTE: You may find std::discrete_distribution helpful here.
	*   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	*/
	vector<Particle> resampled_particles;

	// Create a generator to be used for generating random particle index and beta value
	default_random_engine gen;
	//Generate random particle index
	uniform_int_distribution<int> particle_index(0, num_particles - 1);

	int current_index = particle_index(gen);

	double beta = 0.0;

	double max_weight_2 = 2.0 * *max_element(weights.begin(), weights.end());

	for (int i = 0; i < particles.size(); i++) {
		uniform_real_distribution<double> random_weight(0.0, max_weight_2);
		beta += random_weight(gen);

		while (beta > weights[current_index]) {
			beta -= weights[current_index];
			current_index = (current_index + 1) % num_particles;
		}
		resampled_particles.push_back(particles[current_index]);
	}
	particles = resampled_particles;




}

void ParticleFilter::SetAssociations(Particle& particle,
	const vector<int>& associations,
	const vector<double>& sense_x,
	const vector<double>& sense_y) {
	// particle: the particle to which assign each listed association, 
	//   and association's (x,y) world coordinates mapping
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates
	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();
	particle.associations = associations;
	particle.sense_x = sense_x;
	particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
	vector<int> v = best.associations;
	std::stringstream ss;
	copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1);  // get rid of the trailing space
	return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
	vector<double> v;

	if (coord == "X") {
		v = best.sense_x;
	}
	else {
		v = best.sense_y;
	}

	std::stringstream ss;
	copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1);  // get rid of the trailing space
	return s;
}
