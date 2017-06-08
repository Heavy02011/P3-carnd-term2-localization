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
  for (int i=0; i < num_particles; i++) {

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
    particle.x = dist_x_init(gen); // +=
    particle.y = dist_y_init(gen);// +=
    particle.theta = dist_theta_init(gen); // +=
    
    // add particle to vector of particles
    particles.push_back(particle);
    
    // generate weights vector in addition
    weights.push_back(particle.weight);// remove this later
  
  }
  
  // set initializer flag 
  is_initialized = true;
  
  //checkoutput();
  
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  // TODO: Add measurements to each particle and add random Gaussian noise.
  // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
  //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
  //  http://www.cplusplus.com/reference/random/default_random_engine/

  cout << "ParticleFilter::prediction..." << endl;
  cout << "=======================================" << endl;
  cout << "velocity = " << velocity << " " << yaw_rate << endl;
  cout << "=======================================" << endl;
  cout << endl; 
            
  // generate gaussians
  default_random_engine gen;  
  
  double xf;
  double yf;   
  double thetaf;
  
  // generate noise
/*
  normal_distribution<double> dist_x(0.0, std_pos[0]);
  normal_distribution<double> dist_y(0.0, std_pos[1]);
  normal_distribution<double> dist_theta(0.0, std_pos[2]); 
*/  
  // loop over all particles  
  for (int k=0; k<num_particles; k++) {
  
    // particle state bevor
    double xp = particles[k].x;
    double yp = particles[k].y;
    double thetap = particles[k].theta;
    
    // particle state after prediction
    if (abs(yaw_rate) > 0.00001) { // turning
      
      xf = xp + velocity / yaw_rate * (sin(thetap + yaw_rate * delta_t) - sin(thetap));
      yf = yp + velocity / yaw_rate * (cos(thetap) - cos(thetap + yaw_rate * delta_t));   
      thetaf = thetap + yaw_rate * delta_t;
      
    } else { // moving straight
      
      xf = xp + velocity * delta_t * cos(thetap);
      yf = yp + velocity * delta_t * sin(thetap);   
      thetaf = thetap;

    }    
    // update particles with new values after motion
/*
    particles[k].x = xf;
    particles[k].y = yf;   
    particles[k].theta = thetaf; 
*/    
    // creates a normal (Gaussian) distribution for xf, yf, thetaf
   
    normal_distribution<double> dist_x(xf, std_pos[0]);
    normal_distribution<double> dist_y(yf, std_pos[1]);
    normal_distribution<double> dist_theta(thetaf, std_pos[2]); 
    
    // add noise to particle
    particles[k].x = dist_x(gen);//+=
    particles[k].y = dist_y(gen); //+=
    particles[k].theta = dist_theta(gen); // +=
    
  }
  // checkoutput();

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
  
  cout << "ParticleFilter::updateWeights..." << endl;
      
  // test output of input data
/*
  cout << "sensor_range=" << sensor_range << endl; // 50
  cout << "std_landmark[]=" << std_landmark[0] << " " << std_landmark[1] << endl; // 0.3  0.3
  cout << "observations.size=" << observations.size() << endl; // 11
  cout << "map_landmarks.landmark_list.size()=" << map_landmarks.landmark_list.size() << endl; 
  cout << "weights.size()=" << weights.size() << endl;
  cout << "particles.size()=" << particles.size() << endl;  
  
  cout << endl; 
  cout << "=== MAP_LANDMARKS=======================" << endl;;
  for (int i=0;i<map_landmarks.landmark_list.size();i++) {
    cout << i << " ";
    cout << map_landmarks.landmark_list[i].id_i << " ";
    cout << map_landmarks.landmark_list[i].x_f << " ";
    cout << map_landmarks.landmark_list[i].y_f << endl;
  }  
  cout << "=======================================" << endl;;
  cout << endl; 
  
*/  
  
  // 1 loop over all particles
  // ============================================================================================
  
  for (int k=0; k<num_particles; k++) {
    
    double xp = particles[k].x;
    double yp = particles[k].y;
    double thetap = particles[k].theta;
    //cout << k << "---> " << xp << " " << yp << " " << thetap << endl;
    
    // observations in global map coordinates
    vector<LandmarkObs> observations_gc;
    
    // map landmarks seen by the car
    vector<LandmarkObs> map_landmarks_carsees;   
   
    // precalculate constants
    double costhetap = cos(thetap);
    double sinthetap = sin(thetap);  
    
    // 2 transform observations from vehicle into global map coordinates and store them
    // ============================================================================================
    
    // clear vectors from previous particle
    observations_gc.clear();
    
    for (int i=0; i<observations.size(); i++) { 
      
      // transform from car to map coordinates
      double xg = observations[i].x * costhetap - observations[i].y * sinthetap + xp;
      double yg = observations[i].x * sinthetap + observations[i].y * costhetap + yp;
      
      // generate list of transformed coordinates
      LandmarkObs transformed_observation;
      
      // store the actual values into it
      transformed_observation = {observations[i].id, xg, yg};
      
      // save for later use
      observations_gc.push_back(transformed_observation);
      
    }
    
    // 3 eliminate all landmarks that are beyond sensor_range: compare landmarks to point distances
    // ============================================================================================
    
    // clear vectors from previous particle
    map_landmarks_carsees.clear();
    
    for (int i=0; i<map_landmarks.landmark_list.size(); i++) {
            
      // calculate distance between landmarks and measurements in map coordinates
      double mydist = dist(map_landmarks.landmark_list[i].x_f, map_landmarks.landmark_list[i].y_f, xp, yp);  
            
      if (mydist <= sensor_range) { //#5
        
        // generate landmark object
        LandmarkObs mylandmark;
        
        // store the actual values into it
        mylandmark = {map_landmarks.landmark_list[i].id_i, 
                      map_landmarks.landmark_list[i].x_f, 
                      map_landmarks.landmark_list[i].y_f};
        
        // save for later use
        map_landmarks_carsees.push_back(mylandmark);
        
      }  
      
    }
    
    // 4 associate id of closest landmarks to measurements (observations_gc holds index of associated map_landmarks_carsees)
    // ============================================================================================
    
    
    cout << "ParticleFilter::dataAssociation..." << endl;
  
    double BIGNUMBER = 1.0e100;    
    unsigned int    index4smallestdist;
    double smallestdist;
    
  
    if (map_landmarks_carsees.size() > 0) {
    
      for (int j=0; j<observations_gc.size(); j++) {
        
        double xobs = observations_gc[j].x;      
        double yobs = observations_gc[j].y;
      
        // before restarting for next observation reset minimum distance
        smallestdist = BIGNUMBER; //#1
        
        for (int i=0; i<map_landmarks_carsees.size(); i++) {
          
          double x = map_landmarks_carsees[i].x;
          double y = map_landmarks_carsees[i].y;      
  
          // determine distance  
          double distance = dist(x, y, xobs, yobs); 
          
          if (distance < smallestdist) {
            
            // set the smallest distance
            smallestdist = distance;
            
            // get according index of landmark
            index4smallestdist = i;  //#2
          
          }
          // rbx cout << "j="<<j<<" i="<<i<<" "" << xobs,yobs,xmap,ymap,distance= "<< xobs << " " <<yobs<< " " << x <<" " <<y << " " << distance << " " <<endl;
        
        }
        //rbx cout << "index4smallestdist= " << index4smallestdist << endl; 
        //rbx cout << endl;
         
        //cout << "j="<<j<<" i="<<index4smallestdist<<" "" << xobs,yobs,xmap,ymap,distance= "<< xobs << " " <<yobs<< " " << map_landmarks_carsees[index4smallestdist].x <<" " <<map_landmarks_carsees[index4smallestdist].y << " " << distance << " " <<endl;
        
        // store nearest neighbor
        observations_gc[j].id = index4smallestdist; //#3  #### check this "one" ####
      
      }
     
    }
/*    
    for (int i=0; i<observations_gc.size(); i++) {
      cout << "i= "<<i<<" "<<observations_gc[i].id<<endl;
    }
*/      
    // reinitialze weights
    particles[k].weight = 1.0;
    //weights[k] = 1.0;
      
    // 5 use actual distance between map_landmarks_carsees & observations_gc to update weights
    // ============================================================================================
    
    //cout << "observations_gc.size()=" << observations_gc.size() << endl;
    //cout << "map_landmarks_carsees.size()=" << map_landmarks_carsees.size() << endl;
/*    
            cout << endl; 
            cout << "===OBERSERVATIONS_GC===================" << endl;;
                
            for (int i=0;i<observations_gc.size();i++) {
              cout << observations_gc[i].id << " ";
              cout << observations_gc[i].x << " ";
              cout << observations_gc[i].y << endl;
            }    
            
            cout << "===MAP_LANDMARK_CARSEES================" << endl;;
             for (int i=0;i<map_landmarks_carsees.size();i++) {
              cout << map_landmarks_carsees[i].id << " ";
              cout << map_landmarks_carsees[i].x << " ";
              cout << map_landmarks_carsees[i].y << endl;
            }  
            
            cout << "=======================================" << endl;;
            cout << endl; 
*/    
    double prob = 1.0;
    double sx = std_landmark[0];
    double sy = std_landmark[1];
          
    for (int i=0; i<observations_gc.size(); i++) {
                  
      // id of associated map_landmarks_carsees
      unsigned int iass = observations_gc[i].id;
            cout << "i= "<<i<<" "<<observations_gc[i].id<<" iass="<<iass<<endl;
      //if (iass > 0) {
          
      // local distances between associated landmarks & observations
      double dx = map_landmarks_carsees[iass].x - observations_gc[i].x;
      double dy = map_landmarks_carsees[iass].y - observations_gc[i].y;
      double d = dist(map_landmarks_carsees[iass].x,map_landmarks_carsees[iass].y, observations_gc[i].x,observations_gc[i].y);
              
      // calculate probability
      prob = 1.0; //#4
      double prob1 = 1.0 / (2*M_PI*sx*sy);
      double prob2x = dx * dx / (2.0*sx*sx);
      double prob2y = dy * dy / (2.0*sy*sy);
      double prob2 = -1.0 * (prob2x + prob2y);
      double prob3 = exp(prob2);
      prob = prob1 * prob3;
              
      //}  
      //cout << "k_particle = " << k << " obs: i= " << i << " dx/dy= " << dx << "/" << dy << " dist= " << d << " "<<  observations_gc[i].x << " " << observations_gc[i].y << " map: iass = " << iass << " " << map_landmarks_carsees[iass].x << " " << map_landmarks_carsees[iass].y << " prob = " << prob << endl;   
      //cout << "k_particle = " << k << " obs: i= " << i <<" " <<  observations_gc[i].x << " " << observations_gc[i].y << " map: iass = " << iass << " " << map_landmarks_carsees[iass].x << " " << map_landmarks_carsees[iass].y << " prob = " << prob << endl;   
        
      //rbx cout <<  k << " " << i << " " <<  iass <<  " " << observations_gc[i].x << " " << observations_gc[i].y <<  " " << map_landmarks_carsees[iass].x << " " << map_landmarks_carsees[iass].y << "  " << prob << "  " << d << endl; 
    
      particles[k].weight *= prob;
    }
    weights[k] = particles[k].weight;
    
    // 6 update particle weights
    // ============================================================================================
    //particles[k].weight *= prob;
    //particles[k].weight = prob;    
    //weights[k] = prob; //particles[k].weight; // that cashed the crash
    //rbx cout << "k_particle = " << k << " prob=" << prob << endl;
    
  } 
  
}


void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  cout << "ParticleFilter::resample..." << endl;
  
  // store all weights in one vector
/*
  for (int k=0; k<num_particles; k++) {
    
    weights[k] = particles[k].weight;
    
  } 
*/  
  // generate gaussians
  default_random_engine gen;
  discrete_distribution<> d(weights.begin(), weights.end());
  //cout << weights.begin() << "   "  << weights.end() << endl;
  //map<double, double> particles_new;
  
  // new particles vector
  vector<Particle> particles_new;
  
  // draw new particles from random distribution
  for (int k=0; k<num_particles; k++) {  
    
    particles_new.push_back(particles[d(gen)]);
    
  }
  
  // now set the new particles to the original vector
  particles = particles_new;
  
  //checkoutput();

/*
    // https://discussions.udacity.com/t/resampling-algorithm-using-resampling-wheel/241313/2
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<> d(weights.begin(), weights.end());
    std::map<int, int> m;
    for(int n=0; n<num_particles; ++n) {
        particles_new.push_back(particles[d(gen)]);
    }
*/
  
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

  cout << endl; 
  cout << "===PARTICLES============================" << endl;;
      
   for (int i=0;i<num_particles;i++) {
    cout << particles[i].id << " ";
    cout << particles[i].x << " ";
    cout << particles[i].y << " ";
    cout << particles[i].theta << " ";
    cout << particles[i].weight << endl;
  }     
 
   cout << "=======================================" << endl;;
   cout << endl; 
   
 
 /*  
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
*/
  
}

/*
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
  
  // calling with --> dataAssociation(map_landmarks_carsees, observations_gc); 

  cout << "ParticleFilter::dataAssociation..." << endl;
  
  double BIGNUMBER = 100000000;    
  int    index4smallestdist;
  double smallestdist;
  
  //cout << "observations.size()=" << observations.size() << endl;
  //cout << "predicted.size()=" << predicted.size() << endl; 
            cout << endl; 
            cout << "=== before ============================" << endl;;              
            cout << "===MAP_LANDMARK_CARSEES / predicted ===" << endl;;
             for (int i=0;i<predicted.size();i++) {
              cout << i << " ";
              cout << predicted[i].id << " ";
              cout << predicted[i].x << " ";
              cout << predicted[i].y << endl;
            }  
            cout << "===OBERSERVATIONS_GC / observations ===" << endl;;    
            for (int i=0;i<observations.size();i++) {
              cout << i << " ";
              cout << observations[i].id << " ";
              cout << observations[i].x << " ";
              cout << observations[i].y << endl;
            }  
            
            cout << "=======================================" << endl;;
            cout << endl; 
            

  if (predicted.size() > 0) {
  
    for (int j=0; j<observations.size(); j++) {
      
      double xobs = observations[j].x;      
      double yobs = observations[j].y;
    
      // before restarting for next observation reset minimum distance
      smallestdist = BIGNUMBER; //#1
      
      for (int i=0; i<predicted.size(); i++) {
        
        double x = predicted[i].x;
        double y = predicted[i].y;      

        // determine distance  
        double distance = dist(x, y, xobs, yobs); 
        
        if (distance < smallestdist) {
          
          // set the smallest distance
          smallestdist = distance;
          
          // get according index of landmark
          index4smallestdist = i;  //#2
        
        }
      
      }
      // store nearest neighbor
      observations[j].id = index4smallestdist; //#3
    
    }
    
  }
            cout << endl; 
            cout << "=== after =============================" << endl;;              
            cout << "===MAP_LANDMARK_CARSEES / predicted ===" << endl;;
             for (int i=0;i<predicted.size();i++) {
              cout << i << " ";
              cout << predicted[i].id << " ";
              cout << predicted[i].x << " ";
              cout << predicted[i].y << endl;
            }  
            cout << "===OBERSERVATIONS_GC / observations ===" << endl;;    
            for (int i=0;i<observations.size();i++) {
              cout << i << " ";
              cout << observations[i].id << " ";
              cout << observations[i].x << " ";
              cout << observations[i].y << endl;
            }  
            
            cout << "=======================================" << endl;;
            cout << endl;
            
            
            for (int j=0; j<observations.size(); j++) {
              int iass = observations[j].id;
              cout << "obs: " << observations[j].x << " " << observations[j].y << " map: " << predicted[iass].y << " " << predicted[iass].y << endl;
            }
            
    cout << "ParticleFilter::dataAssociation...finished" << endl; 
    
  
}
*/
