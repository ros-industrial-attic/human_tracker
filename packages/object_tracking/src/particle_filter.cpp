#include <object_tracking/particle_filter.h>


    ParticleFilter::ParticleFilter():
        rng(0xFFFFFFFF)
    {
      initialized = false;
      particle_model.clear();
      N = 500;
      neff = N;
      is_dead_ = false;
    }

    void ParticleFilter::initialize(Particle initial_state)
    {
    	is_dead_ = false;

      // Set MAP to initial contour
      current_map_ = initial_state;

      // Generate initial contour points (our particle model)
      particle_model.clear();
      for (int i = 0; i < N; i++)
      {
        Particle temp;
        temp.x = initial_state.x.clone();
        temp.weight = rng.uniform(0.0,1.0);;
        particle_model.push_back(temp);
      }
      // normalize weights
      double sum = 0.0;
      for(int i = 0; i < N; i++) sum += particle_model[i].weight;
      for(int i = 0; i < N; i++) particle_model[i].weight /= sum;
      initialized = true;
    }

    void ParticleFilter::load_model(const char* perceptionPath)
    {
      cv::FileStorage fs(std::string(std::string(perceptionPath)
      + std::string("/filter.xml")).c_str(),
          cv::FileStorage::READ);
      fs["eigenvectors"] >> A;
      fs["eigenvalues"] >> B;
      fs.release();
    }

    void ParticleFilter::updateState(cv::Mat& input, double dt)
    {
      // resample();
      drift_diffuse_sample(dt);
      observation_density_reweight(input);
      if(!is_dead_)
      {
        double max = 0.0;
        for(int i = 0; i < N; i++)
        {
          if(particle_model[i].weight > max)
          {
            max = particle_model[i].weight;
            current_map_.weight = particle_model[i].weight;
            current_map_.x = particle_model[i].x.clone();
          }
        }
      }
    }

    cv::Mat ParticleFilter::calculate_covariance()
    {
      // Put all points into a single matrix
      cv::Mat allPts(N, current_map_.x.rows, CV_64F); // N-samples, state rows columns
      for(int i = 0; i < N; i++)
        for(int j = 0; j < allPts.cols; j++)
          allPts.at<double>(i,j) = particle_model[i].x.at<float>(j,0);

      // calculate covariance
      cv::Mat covar;
      cv::Mat avg;
      cv::calcCovarMatrix(allPts,covar,avg,cv::COVAR_NORMAL + cv::COVAR_ROWS);
      return covar;
    }

    void ParticleFilter::resample()
    {
      for (int j = 0; j < N; j++)
      {
        // Choose random number from 0 to 1
        double rn = rng.uniform(0.0,1.0);

        // Segment interval from 0 to 1 based on particle weights
        // after picking number, keep subtracting weight and break
        // when subtraction yields number less than zero
        int i;
        for(i = 0; i < N; i++)
        {
          rn = rn - particle_model[i].weight;
          if(rn <= 0.0) break;
        }

        // push each chosen particle back to the list
        Particle temp;
        temp.x = particle_model[i].x.clone();
        temp.weight = particle_model[i].weight;
        particle_model.push_back(temp);
      }
      // Remove original list
      for(int j = 0; j < N; j++) particle_model.erase(particle_model.begin());
    }

    void ParticleFilter::drift_diffuse_sample(double dt)
    {
      if((!A.data)||(!B.data))
        {
        ROS_ERROR("Drift and Diffusion models do not"
            "exist.");
        return;
        }


      // apply x = Ax + Bw to each member of particle_model
      for(int j = 0; j < N; j++)
      {
        // drift
        // particle_model[j].x = A*particle_model[j].x;

        //diffuse
        for(int i = 0; i < B.rows; i++)
          particle_model[j].x += (rng.gaussian(B.at<float>(i,0)*dt)*A.row(i)).t();
      }
    }

    void ParticleFilter::observation_density_reweight(cv::Mat& input)
    {
      // Reweight every particle in model by its observation density
      for(int i = 0; i < N; i++)
      {
        particle_model[i].prevWeight = particle_model[i].weight;
        particle_model[i].weight *= likelihood(input, particle_model[i]);
      }

      // Normalize weights
      double sum = 0.0;
      for(int i = 0; i < N; i++) sum += particle_model[i].weight;
      double normFactor = 1.0/sum;
      if(sum == 0.0) normFactor = 0.0;
      for(int i = 0; i < N; i++) particle_model[i].weight *= normFactor;

      // Compute Neff (effective particle number)
      sum = 0.0;
      for(int i = 0; i < N; i++) sum += pow(particle_model[i].weight, 2);
      neff = 1.0/sum;
      if(sum == 0) neff = 0.0;
       ROS_INFO("Neff = %f", neff);

      if(neff < N * 0.04)
      {
        is_dead_ = true;
      }
      else if(neff < N * 0.75)
      {
        resample();
        observation_density_reweight(input);
      }

    }

    // Calculate likelihood for this particle
    double ParticleFilter::likelihood(cv::Mat& input, Particle states)
    {
      // Virtual function, can be overloaded if more processing is necessary

      int i = states.x.at<float>(1,0);
      int j = states.x.at<float>(0,0);
      if((i < 0)
          ||(j < 0)
          ||(i >= input.rows)
          ||(j >= input.cols))
        return 0.0;
      return input.at<float>(i,j);
    }

