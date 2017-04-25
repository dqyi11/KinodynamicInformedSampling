#ifndef TIME_BENCHMARK_H_
#define TIME_BENCHMARK_H_

#include <chrono>
#include <iostream>
#include <fstream>
#include <vector>

void printTime(std::chrono::high_resolution_clock::duration &duration, 
               std::ostream& cout)
{
    auto duration_s = std::chrono::duration_cast<std::chrono::seconds>(
                      duration).count();
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                       duration).count();
    auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(
                       duration).count();
    if (duration_s != 0)
        cout << duration_s << "s";
    else if (duration_ms != 0)
        cout << duration_ms << "ms";
    else
        cout << duration_us << "us";
}

void printTimeInMs(std::chrono::high_resolution_clock::duration &duration,
               std::ostream& cout)
{
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                       duration).count();

    cout << duration_ms << "ms";
}

void printTimeInUs(std::chrono::high_resolution_clock::duration &duration,
               std::ostream& cout)
{
    auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(
                       duration).count();

    cout << duration_us << "us";
}

void printTimeToFile(std::vector<std::chrono::high_resolution_clock::duration> durations,
                     unsigned int col_num, unsigned int row_num,
                     std::ofstream& fout)
{
    assert(durations.size()==row_num*col_num);
    if (fout.is_open())
    {
        for (unsigned int i = 0; i < row_num; i++)
        {
            for (unsigned int j = 0; j < col_num; j++)
            {
                fout << std::chrono::duration_cast<std::chrono::milliseconds>(durations[i * col_num + j]).count() << " ";
            }
            fout << std::endl;
        }
    }
    fout.close();
}

void appendTimeAndRatioToFile(std::vector<std::chrono::high_resolution_clock::duration> durations,
                              double ratio,
                              int sampleNum,
                              std::ofstream& fout)
{
    for (unsigned int i = 0; i < durations.size(); i++)
    {
        fout << std::chrono::duration_cast<std::chrono::milliseconds>(durations[i]).count() << " ";
    }
    fout << ratio << " " << sampleNum << std::endl;
}

void printTimeToFile(std::vector<std::chrono::high_resolution_clock::duration> durations,
                     std::ofstream& fout)
{
    if (fout.is_open())
    {
        for (unsigned int i = 0; i < durations.size(); i++)
        {

            fout << std::chrono::duration_cast<std::chrono::milliseconds>(durations[i]).count() << std::endl;
        }
    }
    fout.close();
}




#endif // TIME_BENCHMARK_H_
