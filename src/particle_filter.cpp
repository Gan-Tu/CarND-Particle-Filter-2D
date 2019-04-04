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
#include "helper_functions.h"

#define EPS 1E-4

using namespace std;
static default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of  
    //   x, y, theta and their uncertainties from GPS) and all weights to 1. 
    // Add random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    
    this->num_particles = 500;

    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    for (int i = 0; i < this->num_particles; ++i) {
        Particle p;
        p.id = i;
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight = 1.0;
        this->particles.push_back(p);
        weights.push_back(1.0);
    }

    this->is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/

    normal_distribution<double> dist_x(0, std_pos[0]);
    normal_distribution<double> dist_y(0, std_pos[1]);
    normal_distribution<double> dist_theta(0, std_pos[2]);

    for (int i = 0; i < this->num_particles; ++i) {
        if (abs(yaw_rate) < EPS) { // prevent divide by zero
            this->particles[i].x += velocity * delta_t * cos(this->particles[i].theta);
            this->particles[i].y += velocity * delta_t * sin(this->particles[i].theta);
        } else {
            this->particles[i].x += velocity / yaw_rate * (sin(this->particles[i].theta + yaw_rate * delta_t) - sin(this->particles[i].theta));
            this->particles[i].y += velocity / yaw_rate * (cos(this->particles[i].theta) - cos(this->particles[i].theta + yaw_rate * delta_t));
            this->particles[i].theta += yaw_rate * delta_t;
        }
        this->particles[i].x += dist_x(gen);
        this->particles[i].y += dist_y(gen);
        this->particles[i].theta += dist_theta(gen);
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
    //   implement this method and use it as a helper during the updateWeights phase.
    
    for (int i = 0; i < observations.size(); ++i) {

        double min_dist = std::numeric_limits<double>::infinity();
        int min_id = -1;

        for (int j = 0; j < predicted.size(); ++j) {
            double cur_dist = dist( predicted[j].x, predicted[j].y,
                                    observations[i].x, observations[i].y);
            if (cur_dist < min_dist) {
                min_dist = cur_dist;
                min_id = predicted[j].id;
            }
        }

        observations[i].id = min_id;
    }

}

float multi_variate_gaussian( double x, double y, 
                              double mu_x, double mu_y, 
                              double std_x, double std_y) {    
    double gaussian_norm = 1 / (2 * M_PI * std_x * std_y);
    double exponent = - ( pow(x-mu_x, 2) / (2 * pow(std_x, 2)) +
                          pow(y-mu_y, 2) / (2 * pow(std_y, 2)) );
    return gaussian_norm * exp(exponent);
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
        const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
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

    for (int i = 0; i < this->num_particles; ++i) {

        // cache current particle state param
        double p_x = this->particles[i].x;
        double p_y = this->particles[i].y;
        double p_theta = this->particles[i].theta;

        // get predicted landmark positions within the sensor range
        std::vector<LandmarkObs> predicted;
        for (int j = 0; j < map_landmarks.landmark_list.size(); ++j) {
            if (dist(p_x, 
                     p_y, 
                     map_landmarks.landmark_list[j].x_f, 
                     map_landmarks.landmark_list[j].y_f) <= sensor_range) {

                LandmarkObs predicted_landmark;
                predicted_landmark.x = map_landmarks.landmark_list[j].x_f;
                predicted_landmark.y = map_landmarks.landmark_list[j].y_f;
                predicted_landmark.id = map_landmarks.landmark_list[j].id_i;

                predicted.push_back(predicted_landmark);
            }
        }

        // get transformed landmark observations in MAP'S coordinate system
        std::vector<LandmarkObs> observations_transformed;
        for (LandmarkObs obs : observations) {

            LandmarkObs transformed_obs;
            transformed_obs.x = p_x + cos(p_theta) * obs.x - sin(p_theta) * obs.y;
            transformed_obs.y = p_y + sin(p_theta) * obs.x + cos(p_theta) * obs.y;
            transformed_obs.id =  obs.id;

            observations_transformed.push_back(transformed_obs);
        }

        // associate predicted landmark id to each observation
        dataAssociation(predicted, observations_transformed);

        // re-weight particles
        double new_weight = 1.0;

        for (LandmarkObs obs : observations_transformed) {

            // get predicted location for this observed landmark
            double predicted_x, predicted_y;
            for (LandmarkObs p : predicted) {
                if (p.id == obs.id) {
                    predicted_x = p.x;
                    predicted_y = p.y;
                    break;
                }
            }
            
            // get multi-variate gaussian probability
            new_weight *= multi_variate_gaussian(obs.x, obs.y,
                                                 predicted_x, predicted_y,
                                                 std_landmark[0], std_landmark[1]);

        }
        this->particles[i].weight = new_weight;
        weights[i] = new_weight;
    }

}

void ParticleFilter::resample() {
    // TODO: Resample particles with replacement with probability proportional to their weight. 
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    // get all of the current weights
    vector<double> weights;
    for (int i = 0; i < this->num_particles; i++) {
        weights.push_back(this->particles[i].weight);
    }

    std::discrete_distribution<int> weight_dist(weights.begin(), weights.end());

    vector<Particle> new_particles;
    for (int i = 0; i < this->num_particles; i++) {
        new_particles.push_back(this->particles[weight_dist(gen)]);
    }

    this->particles = new_particles;

}

void ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
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
