#ifndef UTIL_H
#define UTIL_H

#include <mug/sample.h>

using namespace mug; 

double __dist(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2)
{
    return sqrt((p1-p2).array().square().sum());
}

Samples filterSamples(const Samples &samples)
{
    Samples filtered;

    Eigen::Vector2f lastTarget;
    int lastOnset = 0;

    for (int i=0; i<samples.size(); i++)
    {
        const Sample &s = samples[i];
        int t = s.timestamp;
        
        if (__dist(s.target_pos, lastTarget) > 10)
        {
            lastOnset = t;
            lastTarget = s.target_pos;
        }

        if (t - lastOnset > 1250)
        {
            filtered.push_back(s);
        }
    }
    return filtered;
}


Samples takeFirstNSeconds(int nsecs, const Samples &samples)
{
    int timeStart = samples[0].timestamp;
    Samples subset;
    for (int i=0; i<samples.size(); i++)
    {
        if (samples[i].timestamp - timeStart < nsecs*1000)
        {
            subset.push_back(samples[i]);
        }
    }
    return subset;
}

Samples takeAfterNSeconds(int nsecs, const Samples &samples)
{
    int timeStart = samples[0].timestamp;
    Samples subset;
    for (int i=0; i<samples.size(); i++)
    {
        if (samples[i].timestamp - timeStart > nsecs*1000)
        {
            subset.push_back(samples[i]);
        }
    }
    return subset;
}

Samples takeRandomSubset(int n, const Samples &samples)
{
    if (n > samples.size())
    {
        n = samples.size();
    }
    std::vector<int> ids;
    for (int i=0; i<samples.size(); i++)
    {
        ids.push_back(i);
    }
    std::random_shuffle(ids.begin(), ids.end());

    Samples subset;
    for (int i=0; i<n; i++)
    {
        subset.push_back(samples[ids[i]]);
    }
    return subset;
}

Samples reduceSampleRate(int n, const Samples &samples)
{
    Samples res;
    for (int i=0; i<samples.size(); i++)
    {
        if (i % n == 0)
            res.push_back(samples[i]);
    }
    return res;
}

#endif
