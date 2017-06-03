/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
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

void ParticleFilter::init(double x, double y, double theta, double std[]) {
// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

  cout << "ParticleFilter::init..." << endl;
    
  // generate gaussians
  default_random_engine gen;

  // This line creates a normal (Gaussian) distribution for x, y, theta
  normal_distribution<double> dist_x_init(x, std[0]);
  normal_distribution<double> dist_y_init(y, std[1]);
  normal_distribution<double> dist_theta_init(theta, std[2]); 
  
  // 1 Set the number of particles
  num_particles = 50;
  
  // 2 Initialize all particles to first position (based on estimates of 
  //   x, y, theta and their uncertainties from GPS) and all weights to 1. 
  for (int i; i < num_particles; i++) {

    // create particle
    Particle particle;  
  
    // set initial values
    particle.id     = i;
    particle.x      = x;
    particle.y      = y;
    particle.theta  = theta;
    particle.weight = 1.0;
  
/* missing initilizations for particles
	std::vector<int> associations;
	std::vector<double> sense_x;
	std::vector<double> sense_y;
*/
  
    // 3 Add random Gaussian noise to each particle. --> https://discussions.udacity.com/t/norm-distribution/249496/3
    particle.x += dist_x_init(gen);
    particle.y += dist_y_init(gen);
    particle.theta += dist_theta_init(gen);
    
    // add particle to vector of particles
    particles.push_back(particle);
  
  }
  
  // set initializer flag 
  is_initialized = true;
    
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  // TODO: Add measurements to each particle and add random Gaussian noise.
  // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
  //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
  //  http://www.cplusplus.com/reference/random/default_random_engine/

  // generate gaussians
  default_random_engine gen;  
  
  // loop over all particles  
  for (int k; k<num_particles; k++) {
  
    // particle state bevor
    double xp = particles[k].x;
    double yp = particles[k].y;
    double thetap = particles[k].theta;
    
    // particle state after prediction
    double xf = xp + velocity / yaw_rate * (sin(thetap + yaw_rate * delta_t) - sin(thetap));
    double yf = yp + velocity / yaw_rate * (cos(thetap) - cos(thetap + yaw_rate * delta_t));   
    double thetaf = thetap + yaw_rate * delta_t;
    
    // update particles with new values after motion
    particles[k].x = xf;
    particles[k].y = yf;   
    particles[k].theta = thetaf; 
    
    // creates a normal (Gaussian) distribution for xf, yf, thetaf
    normal_distribution<double> dist_x(xf, std_pos[0]);
    normal_distribution<double> dist_y(yf, std_pos[1]);
    normal_distribution<double> dist_theta(thetaf, std_pos[2]); 
    
    // add noise to particle
    particles[k].x += dist_x(gen);
    particles[k].y += dist_y(gen);
    particles[k].theta += dist_theta(gen);
    
  }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
      
  // input                                    description
  // --------------------------------------------------------------------------                              
  // double sensor_range                      range of lidar sensor (m)
  // double std_landmark[]                    std deviation of landmark observations in vehicle coordinates
  // std::vector<LandmarkObs> observations    landmark observations
  // Map map_landmarks                        map_landmarks
  
  // test output of input data
  cout << "sensor_range=" << sensor_range << endl; // 50
  cout << "std_landmark[]=" << std_landmark[0] << " " << std_landmark[1] << endl; // 0.3  0.3
  cout << "observations.size=" << observations.size() << endl; // 11
  cout << "map_landmarks.landmark_list.size()=" << map_landmarks.landmark_list.size() << endl; 
    

  // observations in global map coordinates
  vector<LandmarkObs> observations_gc;
  
  // map landmarks seen by the car
  vector<LandmarkObs> map_landmarks_car;
  
  // 1 loop over all particles
  // ============================================================================================
  
  for (int k; k<num_particles; k++) {
    double xp = particles[k].x;
    double yp = particles[k].y;
    double thetap = particles[k].theta;
    //cout << k << "---> " << xp << " " << yp << " " << thetap << endl;
    
    // initialize actual distance between observation and landmark
    double r_obs2landmark;
    r_obs2landmark = sensor_range;
    
    // 2 transform observations from vehicle into global map coordinates and store them
    // ============================================================================================
    
    for (int i; i<observations.size(); i++) { 
      
      // transform from car to map coordinates
      double xg = observations[i].x * cos(thetap) - observations[i].y * sin(thetap) + xp;
      double yg = observations[i].x * sin(thetap) + observations[i].y * cos(thetap) + yp;
      
      // generate list of transformed coordinates
      LandmarkObs transformed_observation;
      
      // store the actual values into it
      transformed_observation = {observations[i].id, xg, yg};
      
      // save for later use
      observations_gc.push_back(transformed_observation);
      
    }
    
    // 3 eliminate all landmarks that are beyond sensor_range
    // ============================================================================================
    
    for (int i; i<map_landmarks.landmark_list.size(); i++) {
      
      // calculate distance between landmarks and measurements in map coordinates
      double dist = sqrt( pow(map_landmarks.landmark_list[i].x_f - observations_gc[i].x,2) + pow(map_landmarks.landmark_list[i].y_f - observations_gc[i].y,2) );  // maybe store that already here to save time!!
      //double distance = dist(map_landmarks_car[i].x, map_landmarks_car[i].y, observations_gc[i].x, observations_gc[i].y);
      
      if (dist < r_obs2landmark) {
        
        // generate list of transformed coordinates
        LandmarkObs mylandmark;
        
        // store the actual values into it
        mylandmark = {map_landmarks.landmark_list[i].id_i, map_landmarks.landmark_list[i].x_f, map_landmarks.landmark_list[i].y_f};
        
        // save for later use
        map_landmarks_car.push_back(mylandmark);
        
      }   
      
    }
    
    // 4 use actual distance between map_landmarks_car & observations_gc to update weights
    // ============================================================================================
    
    double prob = 1;
    
    for (int i; i<map_landmarks_car.size(); i++) {
      
      // local distances
      double dx = map_landmarks_car[i].x - observations_gc[i].x;
      double dy = map_landmarks_car[i].y - observations_gc[i].y;

      // calculate probability
      double sx = std_landmark[0];
      double sy = std_landmark[1];
      double prob1 = 1.0 / (2*M_PI*sx*sy);
      double prob2x = dx * dx / (2.0*sx*sx);
      double prob2y = dy * dy / (2.0*sy*sy);
      double prob2 = -1.0 * (prob2x + prob2y);
      double prob3 = exp(prob2);
      prob = prob1 * prob3;

    }
    
    // 5 update particle weights
    // ============================================================================================
    
    particles[k].weight *= prob;
       
  }
  
  
//################################################################
/*
  cout << endl;
  cout << "map_landmarks.landmark_list.size()=" << map_landmarks.landmark_list.size() << endl; 
  for (int j; j<map_landmarks.landmark_list.size(); j++) {
    cout << "map_landmarks.landmark_list.id_i=" << map_landmarks.landmark_list[j].id_i << endl; 
    cout << "map_landmarks.landmark_list.x_f=" << map_landmarks.landmark_list[j].x_f << endl; 
    cout << "map_landmarks.landmark_list.y_f=" << map_landmarks.landmark_list[j].y_f << endl;  
  }
  cout << "23------------------------------------------------------------------------" << endl;
  cout << endl;
*/  
  
}


void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

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

/******************************************************************************
 *
 * own functions
 *
 */
 
void ParticleFilter::checkoutput() {
  
  ofstream myfile;
  myfile.open ("log.txt");
  
  // check particles    
  myfile << ("ParticleFilter::updateWeights...");
  myfile << "num_particles=" << num_particles << endl;
  
  for (int i=0;i<num_particles;i++) {
    myfile << particles[i].id << " ";
    myfile << particles[i].x << " ";
    myfile << particles[i].y << " ";
    myfile << particles[i].theta << endl;
  }      
  myfile.close();  
  
}
