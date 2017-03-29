#include "Sampler/HitAndRun.h"

const bool VERBOSE = true;

VectorXd GibbsSampler::get_random_sample(double min, double max, const int& dim)
{
  std::uniform_real_distribution<> dis(min, max);
  // Updates the member variable of the class as well
  prev_sample_(dim) = dis(gen_);
  return prev_sample_;
}

MatrixXd GibbsSampler::sample(const int& no_samples, high_resolution_clock::duration& duration)
{
  // Get the limits of the space
  VectorXd max_vals, min_vals;
  const int dim = problem().start_state().size();
  std::tie(max_vals, min_vals) = problem().state_limits();

  // Run until you get the correct number of samples
  MatrixXd samples(no_samples, dim + 1);

  // If you want to time the sampling
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  VectorXd sample;
  unsigned int skip = 0, trys=0;
  for(int i=0; i<no_samples; i++)
  {
    trys=0;
    do
    {
      if (trys > 10000)
      {
        skip++;
        trys=0;
      }
      sample = get_random_sample(min_vals[(i+skip)%dim], max_vals[(i+skip)%dim], (i+skip)%dim);
      trys++;
      // if (VERBOSE) std::cout << "Trys:" << trys << " Skip:" << skip << std::endl;
    }
    while(!problem().is_in_level_set(sample));
    VectorXd newsample(problem().start_state().size() + 1);
    prev_sample_ = sample;
    newsample << sample, problem().get_cost(sample);
    samples.row(i) = newsample;
  }

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration = t2 - t1;
  return samples;
}

void GibbsSampler::update_level_set(const double& level_set)
{
  prev_sample_ = problem_.start_state();
  problem_.update_level_set(level_set);
  // std::cout << "Updated Level Set" << std::endl;
}

MatrixXd HitAndRun::sample(const int& no_samples, high_resolution_clock::duration& duration)
{
  // Get the limits of the space
  VectorXd max_vals, min_vals;
  const int dim = problem().start_state().size();
  std::tie(max_vals, min_vals) = problem().state_limits();
  double diag = 0;
  for (unsigned int i=0; i<dim; i++)
    diag = diag + (max_vals[i]-min_vals[i]) * (max_vals[i]-min_vals[i]);
  diag = std::sqrt(diag);

  // Run until you get the correct number of samples
  MatrixXd samples(no_samples, dim + 1);

  // Set up RGN
  std::normal_distribution<> norm_dis(0, 1);
  double lamda_upper_bound, lamda_lower_bound;
  VectorXd dir(dim);

  // If you want to time the sampling
  high_resolution_clock::time_point t1 = high_resolution_clock::now();

  VectorXd sample;
  int skip = 0, trys = 0;
  for(int i=0; i<no_samples; i++)
  {
    trys=-1;
    do
    {
      if (trys > 10000 || trys == -1)
      {
        // Sample random direction in S^dim
        double sum=0;
        for (unsigned int i=0; i<dim; i++)
          {
            dir[i] = norm_dis(gen_);
            sum = sum + dir[i]*dir[i];
          }
        dir = dir / sum;
        lamda_upper_bound = diag;
        lamda_lower_bound = -diag;
        skip++;
        trys=0;
      }
      // Generate random sample along dir
      std::uniform_real_distribution<> uni_dis(lamda_lower_bound, lamda_upper_bound);
      double lamda = uni_dis(gen_);
      sample = prev_sample_ + lamda*dir;
      if (!problem().is_in_bound(sample))
      {
        if (abs(diag - lamda) < abs(-diag - lamda))
          lamda_upper_bound = lamda;
        else
          lamda_lower_bound = lamda;
      }
      trys++;
      if (VERBOSE) std::cout << "Current_sample: " << i << " Trys: " << trys << " Skip: " << skip
                             << " UB:" << lamda_upper_bound << " LB:" << lamda_lower_bound << std::endl;
    }
    while(!problem().is_in_level_set(sample));
    VectorXd newsample(problem().start_state().size() + 1);
    prev_sample_ = sample;
    newsample << sample, problem().get_cost(sample);
    samples.row(i) = newsample;
  }

  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  duration = t2 - t1;
  return samples;
}
