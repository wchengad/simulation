#include "common_utility.h"


int q2r(const float *pq, float *pr)
{
    float x, y, z, r;

    r = pq[0]; x = pq[1]; y = pq[2]; z = pq[3];
    pr[0] = r*r+x*x-y*y-z*z;
    pr[1] = 2*(x*y-r*z);
    pr[2] = 2*(z*x+r*y);

    pr[3] = 2*(x*y+r*z);
    pr[4] = r*r-x*x+y*y-z*z;
    pr[5] = 2*(y*z-r*x);

    pr[6] = 2*(z*x-r*y);
    pr[7] = 2*(y*z+r*x);;
    pr[8] = r*r-x*x-y*y+z*z;

    return 0;
}

int r2e(const float *pr, float *pe)
{
    float ex, ey, ez;

    ez = atan2(pr[3], pr[0]);
    ey = asin(-pr[6]);
    ex = atan2(pr[7], pr[8]);

    pe[0] = ez;		//yaw
    pe[1] = ey;		//pitch
    pe[2] = ex;		//roll

    return 0;
}

int q2e(const float *pq, float *pe)
{
    float rr[3*3];

    q2r(pq, rr);
    r2e(rr, pe);

    return 0;
}
///(0,0) (1,0) (2,0) (0,1) (1,1) (2,1) (0,2) (1,2) (2,2)
///0     1     2     3     4     5     6     7     8
int r2q(const float *pr, float *pq)
{
    double  tr = pr[0] + pr[4] + pr[8];
    if (tr > 0)
    {
        double S = sqrt(tr + 1.0) * 2;
        pq[0] = 0.25 * S;
        pq[1] = (pr[5] - pr[7]) / S;
        pq[2] = (pr[6] - pr[2]) / S;
        pq[3] = (pr[1] - pr[3]) / S;
    }
    else if (pr[0] > pr[4] && pr[0] > pr[8])
    {
        double S = sqrt(1.0 + pr[0] - pr[4] - pr[8]) * 2;
        pq[0] = (pr[5] - pr[7]) / S;
        pq[1] = 0.25 * S;
        pq[2] = (pr[3] + pr[1]) / S;
        pq[3] = (pr[6] + pr[2]) / S;
    }
    else if (pr[4] > pr[8])
    {
        double S = sqrt(1.0 + pr[4] - pr[0] - pr[8]) * 2;
        pq[0] = (pr[6] - pr[2]) / S;
        pq[1] = (pr[3] + pr[1]) / S;
        pq[2] = 0.25 * S;
        pq[3] = (pr[7] + pr[5]) / S;
    }
    else
    {
        double S = sqrt(1.0 + pr[8] - pr[0] - pr[4]) * 2;
        pq[0] = (pr[1] - pr[3]) / S;
        pq[1] = (pr[6] + pr[2]) / S;
        pq[2] = (pr[7] + pr[5]) / S;
        pq[3] = 0.25 * S;
    }
    return 0;
}
