#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include "pose_utils.h"

class LinearTrajectoryGenerator
{
  public:
  
    LinearTrajectoryGenerator()
    {
      initp = zeros<colvec>(3);
      waypoints.clear();      
      ts.clear();
      maxVel = 1;
      maxAcc = 1;
    }
    
    ~LinearTrajectoryGenerator()
    {
    }
    
    void SetVelAccLimit(double _maxVel, double _maxAcc)
    {
      maxVel = _maxVel;
      maxAcc = _maxAcc;
    }
    
    void SetWaypoints(const colvec& _initp, const ros::Time& _initt, const vector<colvec>& _waypoints)
    {
      initp     = _initp;
      initt     = _initt;
      waypoints = _waypoints;
      ts.clear();
      // Calculate time to reach each waypoint
      double accT = maxVel / maxAcc;
      double accD = maxAcc * accT * accT / 2; 
      ros::Time t = initt;           
      for (unsigned int k = 0; k < waypoints.size(); k++)
      {
        colvec p0 = (k == 0)?initp:waypoints[k-1];  
        colvec p1 = waypoints[k];
        double D  = norm(p1 - p0, 2);
        if (D < 2 * accD)
          t += ros::Duration(sqrt(D / maxAcc)) + ros::Duration(sqrt(D / maxAcc));
        else
          t += ros::Duration(accT) + ros::Duration((D - 2*accD) / maxVel) + ros::Duration(accT);
        ts.push_back(t);
      }
    }
    
    void GetPositionCommand(ros::Time t, quadrotor_msgs::PositionCommand& cmd)
    {
      // Init cmd to zero
      cmd.position.x     = 0;
      cmd.position.y     = 0;
      cmd.position.z     = 0;    
      cmd.velocity.x     = 0;
      cmd.velocity.y     = 0;
      cmd.velocity.z     = 0;    
      cmd.acceleration.x = 0;
      cmd.acceleration.y = 0;
      cmd.acceleration.z = 0;  
      cmd.yaw            = 0;
      cmd.yaw_dot        = 0;        
    
      // Find current waypoint
      unsigned int idx = 0;
      for (; idx < ts.size(); idx++)
      {
        double dt = (ts[idx] - t).toSec();
        if (dt >= 0)
          break;
        if (idx == ts.size() - 1)
          break;
      }
      colvec    p0 = (idx == 0)?initp:waypoints[idx-1];
      colvec    p1 = waypoints[idx];
      ros::Time t0 = (idx == 0)?initt:ts[idx-1];
    
      // Pre-compute segment acc/de-acc time and dist
      colvec d       = p1 - p0;
      double D       = norm(d, 2);
      colvec vect    = d / D; 
      double accT    = maxVel / maxAcc;
      double accD    = maxAcc * accT * accT / 2;
      if (D < 2 * accD)         // Short segment, always acc or de-acceleration
      {
        ros::Time t1 = t0 + ros::Duration(sqrt(D / maxAcc));
        ros::Time t2 = t1 + ros::Duration(sqrt(D / maxAcc));
        if ((t1 - t).toSec() > 0)       // Acc    
        {
          colvec acc = maxAcc * vect;
          colvec vel = maxAcc * (t - t0).toSec() * vect;
          colvec pos = maxAcc * (t - t0).toSec() * (t - t0).toSec() / 2 * vect + p0;
          cmd.position.x     = pos(0);
          cmd.position.y     = pos(1);
          cmd.position.z     = pos(2);
          cmd.velocity.x     = vel(0);
          cmd.velocity.y     = vel(1);
          cmd.velocity.z     = vel(2);                    
          cmd.acceleration.x = acc(0);
          cmd.acceleration.x = acc(1);
          cmd.acceleration.x = acc(2);          
        }
        else if ((t2 - t).toSec() > 0)  // De-Acc
        { 
          colvec acc = -maxAcc * vect;
          colvec vel =  maxAcc * (t2 - t).toSec() * vect;
          colvec pos =  (D - maxAcc * (t2 - t).toSec() * (t2 - t).toSec() / 2) * vect + p0;
          cmd.position.x     = pos(0);
          cmd.position.y     = pos(1);
          cmd.position.z     = pos(2);
          cmd.velocity.x     = vel(0);
          cmd.velocity.y     = vel(1);
          cmd.velocity.z     = vel(2);                    
          cmd.acceleration.x = acc(0);
          cmd.acceleration.x = acc(1);
          cmd.acceleration.x = acc(2);            
        }
        else                            // Reach goal
        {
          cmd.position.x = p1(0);
          cmd.position.y = p1(1);
          cmd.position.z = p1(2);           
        }
                    
      }
      else                      // Long segment, has constant velocity profile
      {
        ros::Time t1 = t0 + ros::Duration(accT);
        ros::Time t2 = t1 + ros::Duration((D - 2*accD) / maxVel);
        ros::Time t3 = t2 + ros::Duration(accT);
        if ((t1 - t).toSec() > 0)       // Acc
        {
          colvec acc = maxAcc * vect;
          colvec vel = maxAcc * (t - t0).toSec() * vect;
          colvec pos = maxAcc * (t - t0).toSec() * (t - t0).toSec() / 2 * vect + p0;
          cmd.position.x     = pos(0);
          cmd.position.y     = pos(1);
          cmd.position.z     = pos(2);
          cmd.velocity.x     = vel(0);
          cmd.velocity.y     = vel(1);
          cmd.velocity.z     = vel(2);                    
          cmd.acceleration.x = acc(0);
          cmd.acceleration.x = acc(1);
          cmd.acceleration.x = acc(2);                    
        }
        else if ((t2 - t).toSec() > 0)  // Const speed
        {
          colvec vel = maxVel * vect;
          colvec pos = (accD + maxVel * (t - t1).toSec()) * vect + p0;
          cmd.position.x     = pos(0);
          cmd.position.y     = pos(1);
          cmd.position.z     = pos(2);
          cmd.velocity.x     = vel(0);
          cmd.velocity.y     = vel(1);
          cmd.velocity.z     = vel(2);            
        }
        else if ((t3 - t).toSec() > 0)  // De-Acc
        {
          colvec acc = -maxAcc * vect;
          colvec vel = (maxVel - maxAcc * (t - t2).toSec()) * vect;
          colvec pos = (accD + D - 2*accD + maxVel * (t - t2).toSec() - maxAcc * (t - t2).toSec() * (t - t2).toSec() / 2) * vect + p0;
          cmd.position.x     = pos(0);
          cmd.position.y     = pos(1);
          cmd.position.z     = pos(2);
          cmd.velocity.x     = vel(0);
          cmd.velocity.y     = vel(1);
          cmd.velocity.z     = vel(2);                    
          cmd.acceleration.x = acc(0);
          cmd.acceleration.x = acc(1);
          cmd.acceleration.x = acc(2);           
        }
        else                            // Reach goal
        {
          cmd.position.x = p1(0);
          cmd.position.y = p1(1);
          cmd.position.z = p1(2);            
        } 
      }
    }
  
  private:
    colvec            initp;      // Initial position
    ros::Time         initt;      // Initial time
    vector<colvec>    waypoints;  // Waypoints
    vector<ros::Time> ts;         // Time to reach each waypoint
    double maxVel;
    double maxAcc;
};

