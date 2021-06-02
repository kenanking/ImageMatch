#ifndef TIC_TOC_H_
#define TIC_TOC_H_

#include <ctime>
#include <cstdlib>
#include <chrono>

class TicToc
{
public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count();
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

#endif // TIC_TOC_H_