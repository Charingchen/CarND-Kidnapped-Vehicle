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

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;
using std::default_random_engine;
default_random_engine gen;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 50;  // TODO: Set the number of particles
  default_random_engine gen;
  normal_distribution<double> dist_x(x,std[0]);
  normal_distribution<double> dist_y(y,std[1]);
  normal_distribution<double> dist_theta(theta,std[2]);

  for (int i = 0; i < num_particles; ++i){
      Particle temp_particle;
      temp_particle.id = i;
      temp_particle.x = dist_x(gen);
      temp_particle.y = dist_y(gen);
      temp_particle.theta = dist_theta(gen);
      temp_particle.weight = 1;
      particles.push_back(temp_particle);
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
    double x_p,y_p,theta_p;
    
    for (int i = 0; i < num_particles; ++i) {
        double theta = particles[i].theta;
        double v_over_yawrate = velocity/yaw_rate;
        double t_times_yawrate = yaw_rate * delta_t;
        x_p = particles[i].x + v_over_yawrate * (sin(theta + t_times_yawrate) - sin(theta));
        y_p = particles[i].y + v_over_yawrate * (cos(theta) - cos(theta + t_times_yawrate));
        theta_p = theta + t_times_yawrate;
        normal_distribution<double> dist_x_p(x_p,std_pos[0]);
        normal_distribution<double> dist_y_p(y_p,std_pos[1]);
        normal_distribution<double> dist_theta_p(theta_p,std_pos[2]);
        
        // Record back to particles struct
        particles[i].x = dist_x_p(gen);
        particles[i].y = dist_y_p(gen);
        particles[i].theta = dist_theta_p(gen);
        
    }

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
    for (int k = 0; k < observations.size(); ++k){
        double o_x = observations[k].x;
        double o_y = observations[k].y;
        // Assign the first dist of precticted and observations
        double min_dist = dist(o_x, o_y, predicted[0].x, predicted[0].y);
        int id = predicted[0].id;
        
        for (int i = 1; i < predicted.size();++i){
            double temp_min = dist(o_x, o_y, predicted[i].x, predicted[i].y);
            if (temp_min < min_dist){
                min_dist = temp_min;
                id = predicted[i].id;
            }
        }
        // assign the the id to observation
        observations[k].id = id;
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
    
    // Scan through each particals
    for (int i = 0;i < particles.size(); ++i){
        // First, store particle's values to local variables to reduce runtime
        double x_p = particles[i].x;
        double y_p = particles[i].y;
        double theta_p = particles[i].theta;
        // Perform Calculation only once to save runtime
        double sin_theta_p = sin(theta_p);
        double cos_theta_p = cos(theta_p);
        
        // Translate observations into maps coordinates respect to the current partical
        
        vector<LandmarkObs> observations_t; // Declare translated observation to hold result
        // Loop through orginial observations list
        for (int j = 0; j < observations.size();++j){
            double o_x = observations[j].x;
            double o_y = observations[j].y;
            double m_x_t = x_p + cos_theta_p * o_x - sin_theta_p * o_y;
            double m_y_t = y_p + sin_theta_p * o_x + cos_theta_p * o_y;
            observations_t.push_back(LandmarkObs{observations[j].id, m_x_t,m_y_t});
        }
        
        // Prectied is the map landmarks within sensor range respect to current partical's location.
        vector<LandmarkObs> predicted; // Declare predicted as Landmark objects
        // if the landmark is within sensor range , append to predicted
        for (int k = 0; k < map_landmarks.landmark_list.size(); ++k){
            // Calculate the radius of circle using particle x,y as the center and x y value of map landmarks to determine if those are within the sensor range
            
            double x_f = map_landmarks.landmark_list[k].x_f;
            double y_f = map_landmarks.landmark_list[k].y_f;
            
            double landmark_dist = dist(x_f, y_f, x_p, y_p);
            
            if ( landmark_dist <= sensor_range){
                LandmarkObs temp_landmark;
                temp_landmark.x = x_f;
                temp_landmark.y = y_f;
                temp_landmark.id = map_landmarks.landmark_list[k].id_i;
                predicted.push_back(temp_landmark);
            }
//            double p_radius_sq = pow(x_f - x_p, 2.0) + pow(y_f - y_p, 2.0);
//            double sensor_range_radius = sensor_range/2.0;
//            if(p_radius_sq < (sensor_range_radius * sensor_range_radius)){
//                LandmarkObs temp_landmark;
//                temp_landmark.x = x_f;
//                temp_landmark.y = y_f;
//                temp_landmark.id = map_landmarks.landmark_list[k].id_i;
//                predicted.push_back(temp_landmark);
//            }
        }
        // call dataAssiation add sort ID
        dataAssociation(predicted, observations_t);
        
        // Loop through observation again to calculate Multivariate-Gaussian probability density
        double total_weight = 1.0;
        
        for (int j = 0; j < observations_t.size(); ++j) {
            int index = observations_t[j].id - 1;
            double mu_x = map_landmarks.landmark_list[index].x_f;
            double mu_y = map_landmarks.landmark_list[index].y_f;
            
            total_weight *= multiv_prob(std_landmark, observations_t[j].x, observations_t[j].y, mu_x, mu_y);
        }
        particles[i].weight = total_weight;
    }
    

}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
    vector<double> weights;
    for(int i = 0; i < particles.size();++i){
        weights.push_back(particles[i].weight);
    }
    std::discrete_distribution<double> distribution(weights.begin(),weights.end());
    
//    std::cout << "Probbility of particles"<< std::endl;
//    for (double x:distribution.probabilities()) std::cout << x << " ";
//
//    for (int j = 0; j < 10; ++j){
//         std::cout << "j ="<< j << "  dist return"<< distribution(gen)<<std::endl;
//
//    }
    vector<Particle> resampled_particles;
    
    for(int i = 0; i < particles.size();++i){
        resampled_particles.push_back(particles[distribution(gen)]);
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
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
