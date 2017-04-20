#ifndef SAMPLE_BENCHMARK_H_
#define SAMPLE_BENCHMARK_H_

#include <fstream>
#include <Eigen/Dense>

void printSampleToFile(Eigen::MatrixXd& samples,
                     std::ofstream& fout)
{
    if (fout.is_open())
    {
        for (int i = 0; i < samples.rows(); i++)
        {
            fout << samples.row(i) << std::endl;
        }
    }
    fout.close();
}

#endif //SAMPLE_BENCHMARK_H_
