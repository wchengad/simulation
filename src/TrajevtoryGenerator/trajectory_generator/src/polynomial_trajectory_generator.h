#ifndef POLYNOMIAL_TRAJECTORY_GENERATOR_H
#define POLYNOMIAL_TRAJECTORY_GENERATOR_H

#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include "pose_utils.h"

class PolynomialTrajectoryGenerator
{
  public:
  
    PolynomialTrajectoryGenerator()
    {
      initp = zeros<colvec>(3);
      waypoints.clear();      
      ts.clear();
      vel     = 1;          // Maximum speed 1m/s
      acc     = 1;          // Maximum acceleration 1m/s^2
      vel_yaw = 30*M_PI/180;  // Maximum yaw velocity 30 deg/s
      R       = 3;          // 3th order, Minimum jerk
      N       = 2*R-1; 
      Ryaw    = 2;          // 2nd order, minimum angular acceleration
      Nyaw    = 2*Ryaw-1;
      ishoverxyz    = false;
      ishoveryaw    = false;
      hoverwaypoint = zeros<colvec>(4);
      trajectory_id = 0;
    }
    
    ~PolynomialTrajectoryGenerator() 
    { 
    }
    
    void SetVelocity(double _vel) 
    { 
      vel  = _vel; 
    }
    
    void SetAcceleration(double _acc) 
    { 
      acc  = _acc; 
    }
    
    void SetYawVelocity(double _vel_yaw)
    {
      vel_yaw = _vel_yaw;
    }
    
    void SetDerivativeOrder(double _devOrder)
    {
      R    = (_devOrder >= 2)?_devOrder:2; // Minimum order is 2, minimum acceleration trajectory
      N    = 2*R-1;        
    }
    
    void SetYawDerivativeOrder(double _devOrder)
    {
      Ryaw = (_devOrder >= 2)?_devOrder:2; // Minimum order is 2, minimum angular acceleration, otherwise there is nothing to optimize...
      Nyaw = 2*Ryaw-1;          
    }

    //NOTE_XL: make traj here every time new points come in
    void SetWaypoints(const colvec& _initp, const colvec& _initv, const colvec& _inita, 
                      const double& _inityaw, const double& _initdyaw,
                      const ros::Time& _initt, 
                      const vector<colvec>& _waypoints, bool reuse_t = false)
    {
      static double test_t_xl = 0;
//      ROS_INFO("xl_test_SetWaypoints: delta_t = %lf....... \n", _initt.toSec() - test_t_xl);
//      ROS_INFO("xl_test_SetWaypoints: reuse_t = %d....... \n", reuse_t);
      test_t_xl = _initt.toSec();

      // Init
      polyx.clear();
      polyy.clear();
      polyz.clear();    
      polyyaw.clear();  
      initp     = _initp;
      initv     = _initv;
      inita     = _inita;
      inityaw   = _inityaw;
      initdyaw  = _initdyaw;
      initt     = _initt;
      vector<colvec> waypointsprev = waypoints;
      waypoints = _waypoints;   

      // update trajectory id
      ++trajectory_id;
      
      // De-normalize yaw, using init yaw as reference
      double curryaw = inityaw; 
      for (unsigned int k = 0; k < waypoints.size(); k++)
      {
        double diffyaw  = waypoints[k](3) - curryaw;
        while (diffyaw >  M_PI)
          diffyaw -= 2 * M_PI;
        while (diffyaw < -M_PI)
          diffyaw += 2 * M_PI;
        curryaw += diffyaw;
        waypoints[k](3) = curryaw;
      }
      
      // Special case: hover, one segment only, separate for xyz and yaw
      ishoverxyz = false;
      ishoveryaw = false;      
      if (waypoints.size() == 1)
      {
//        ROS_INFO("xl_test_SetWaypoints: waypoints.size() == 1... \n");
        //NOTE_XL: the case for one segment only

        if (norm(waypoints[0].rows(0,2) - initp, 2) < HOVER_D)
        {
          //ROS_ERROR("Hover:  XYZ");                  
          ishoverxyz = true;
        }
        if (fabs(waypoints[0](3) - inityaw) < HOVER_YAW)      
        {
          //ROS_ERROR("Hover:  Yaw");                            
          ishoveryaw = true;
        }
        if (ishoverxyz || ishoveryaw)
          hoverwaypoint = waypoints[0];
      }
      
      // Re-generating means that we still try to reach the waypoints "ontime", try reuse previous waypoint times
      //NOTE_XL: reuse_t = 0 in our cases
      if (reuse_t)
      {
        // Recompute if the previous trajectory time was passed
        ts.erase(ts.begin(), ts.begin()+(ts.size()-waypoints.size()));
        if ((ts.back() - initt).toSec() <= 0)
        {
          ROS_ERROR("Recompute Time:  Final Waypoint Passed");
          reuse_t = false;       
        }
        // Handle first segment, recompute time if either position or yaw jumps too much
        waypointsprev.erase(waypointsprev.begin(), waypointsprev.begin()+(waypointsprev.size()-waypoints.size()));
        colvec dprev = waypointsprev[0].rows(0,2) - initp;
        colvec dcurr = waypoints[0].rows(0,2)     - initp;     
        if (norm(dcurr, 2) / norm(dprev, 2) > REUSE_T_RATIO && norm(waypoints[0].rows(0,2) - initp, 2) > HOVER_D)
        {
          ROS_ERROR("Recompute Time:  Position Jump:  %f  %f", norm(dprev, 2), norm(dcurr, 2));
          reuse_t = false;
        }        
        double dyawprev = waypointsprev[0](3)     - inityaw;
        double dyawcurr = waypoints[0](3)         - inityaw;   
        while (dyawprev >  M_PI)
          dyawprev -= 2 * M_PI;
        while (dyawcurr < -M_PI)
          dyawcurr += 2 * M_PI;   
        if (fabs(dyawcurr) / fabs(dyawprev) > REUSE_T_RATIO && fabs(waypoints[0](3) - inityaw) > HOVER_YAW)
        {
          ROS_ERROR("Recompute Time:  Yaw Jump:  %f  %f", dyawprev*180/M_PI, dyawcurr*180/M_PI);
          reuse_t = false;
        }                               
      }
      
      /**/
      // Generate new time to waypoints
      if (!reuse_t)
      {
        // Get segment times      
        ts.clear();
        for (unsigned int k = 0; k < waypoints.size(); k++)
        {
          // Rotation time
          double Dyaw  = fabs( waypoints[k](3) - ((k == 0)?inityaw:waypoints[k-1](3)) );
          double dtyaw = Dyaw / vel_yaw;        
          // Position time
          double dtxyz;
            
          colvec p0   = (k == 0)?initp:waypoints[k-1].rows(0,2);    // The start point of this segment
          colvec p1   = waypoints[k].rows(0,2);                     // The end point of this segment

          colvec v0   = (k == 0)?initv:zeros<colvec>(3);            // The init velocity

          colvec d    = p1 - p0;                                    // The position difference
          double D    = (norm(d, 2) == 0)?EPSILON_D:norm(d, 2);     // The norm of position difference 

          double V0   = dot(v0, d / D);                             // Map velocity to the vector (p1-p0)

          double aV0  = fabs(V0);                                   // The absolote mapped velocity

          double acct = (vel - V0) / acc * ((vel > V0)?1:-1);       // The time to speed up to to the max veloctiy

          double accd = V0 * acct + (acc * acct * acct / 2) * ((vel > V0)?1:-1);    // The distance to speed up
          double dcct = vel / acc;                                  // The time to slow down.
          double dccd = acc * dcct * dcct / 2;                      // The distance to slow down.

          if (D < aV0 * aV0 / (2 * acc)) // if not enough distance to slow down
          {
            double t1 = (V0 < 0)?2.0 * aV0 / acc:0.0;
            double t2 = aV0 / acc;
            dtxyz     = t1 + t2;                 
          }
          else if (D < accd + dccd)    // if not enough distance to speed up and slow dwon 
          {                
            double t1 = (V0 < 0)?2.0 * aV0 / acc:0.0;
            double t2 = (-aV0 + sqrt(aV0 * aV0 + acc * D - aV0 * aV0 / 2)) / acc;
            double t3 = (aV0 + acc * t2) / acc;
            dtxyz     = t1 + t2 + t3;    
          }
          else                        // can reach max veloctiy and keep for a while.
          {                 
            double t1 = acct;                              
            double t2 = (D - accd - dccd) / vel;
            double t3 = dcct;
            dtxyz     = t1 + t2 + t3;                                                                  
          }
          // Use the longer time for segment time 
          double dt = (dtxyz > dtyaw)?dtxyz:dtyaw;
          // FIXME: Special case, one short segment, increase time
          if (waypoints.size() == 1 && D < accd + dccd)
          {
            ROS_WARN("Single Short Segment:  %f / %f, time: %f", D, accd + dccd, dt);
            ROS_WARN("p0(%.2f,%.2f)->p1(%.2f,%.2f)", p0(0), p0(1), p1(0), p1(1));          
            dt *= 2;
          } 
          ts.push_back( ((k == 0)?initt:ts.back()) + ros::Duration(dt) );                                 
        }
      }           
       
      // Calculate polynomial coefficients for position one axis at a time      
      for (unsigned int axis = 0; axis < 3; axis++)
      {
        mat    Q = zeros<mat>((N+1)*waypoints.size() , (N+1)*waypoints.size()); // Hessian Matrix
        mat    A = zeros<mat>((N+1)*waypoints.size() , (N+1)*waypoints.size()); // A(Mapping Matrix) P(polynomial coeff) = D(polynomial derivatives)
        mat    M = zeros<mat>(2*R+2*(waypoints.size()-1)+(R-1)*(waypoints.size()-1) , (N+1)*waypoints.size()); //
        colvec b = zeros<colvec>((N+1)*waypoints.size()); //Derivatives
        // for Bookeeping M matrix...
        
        unsigned int fixCnt    = R                   /*start, first seg*/          + R                    /*end, last seg*/         + 
                                (waypoints.size()-1) /* start, except first seg */ + (waypoints.size()-1) /* end, except last seg */; // The fixed part of derivatives.
        unsigned int fixRow    = 0;  
        unsigned int fixCol    = 0;    // Position (0th dev) of start of first seg
        unsigned int floatRow  = fixCnt;  
        unsigned int floatCol  = R+1;  // Velocity (1th dev) of end   if first seg

        for (unsigned int k = 0; k < waypoints.size(); k++)
        {       
          // Start & End point, time
          colvec    p0 = (k == 0)?initp:waypoints[k-1].rows(0,2);
          colvec    v0 = (k == 0)?initv:zeros<colvec>(3);          
          colvec    a0 = (k == 0)?inita:zeros<colvec>(3);                    
          colvec    p1 = waypoints[k].rows(0,2);
          ros::Time t0 = (k == 0)?initt:ts[k-1];      
          ros::Time t1 = ts[k];
          double    dt = (t1-t0).toSec();         
          // Construct cost function
          mat _Q = zeros<mat>(N+1, N+1);
          for (int i = 0; i <= N; i++)
          {
            for (int l = 0; l <= N; l++)
            {
              if (i >= R && l >= R)
              {
                _Q(i,l) = 2.0 * pow(dt, i+l-2*R+1) / (i+l-2*R+1);
                for (int m = 0; m <= R-1; m++)
                  _Q(i,l) *= (i-m) * (l-m);
              }
            }
          }
          /* J=c(3)*[ 6*a(3)*dt+12*a(4)*dt^2+20*a(4)*dt^3 ]
             Q=
             [  0   0   0   0   0   0
                0   0   0   0   0   0
                0   0   0   0   0   0

                0   0   0   dt      dt^2    dt^3
                0   0   0   dt^2    dt^3    dt^4
                0   0   0   dt^3    dt^4    dt^5
             ]
           */
          // Construct constraints A
          mat _A0 = eye<mat>(R, N+1);
          for (int r = 0; r <= R-1; r++)
            for (int m = 0; m <= r-1; m++)
              _A0(r,r) *= r - m;
          
          mat _A1 = zeros<mat>(R, N+1);
          for (int r = 0; r <= R-1; r++)
          {
            for (int n = r; n <= N; n++)
            {
              _A1(r,n) = pow(dt, n-r);
              for (int m = 0; m <= r-1; m++)
                _A1(r,n) *= n - m;
            }
          }
          mat _A = join_cols(_A0, _A1);
          //Construct constraints b
          colvec _b = zeros<colvec>(N+1);
          _b(0)     = p0(axis);
          _b(1)     = v0(axis); // Init velocity of the start of the first segment
          if (R > 2)            // Only allow init acceleration if it is not the objective cost function
            _b(2)     = a0(axis);
          _b(R)     = p1(axis);
          // Construct full matrices
          Q.submat(k*(N+1), k*(N+1), (k+1)*(N+1)-1, (k+1)*(N+1)-1) = _Q;
          A.submat(k*(N+1), k*(N+1), (k+1)*(N+1)-1, (k+1)*(N+1)-1) = _A;
          b.rows(k*(N+1), (k+1)*(N+1)-1)                           = _b;                  
          // Construct mapping matrix
          // Start Point
          if (k == 0)
          {
            // Fix
            for (int i = 0; i < R; i++)
            {
              M(fixRow, fixCol) = 1;
              fixRow++;
              fixCol++;
            }
            // No Float
          }
          else
          {
            // 1 Fix
            M(fixRow, fixCol) = 1;
            fixRow++;
            fixCol++;
            fixCol += R-1;
            // R-1 Float
            for (int i = 0; i < R-1; i++)
            {
              M(floatRow, floatCol) = 1;
              floatRow++;
              floatCol++;
            }
            floatCol += 1;
          }
          // End Point
          if (k == waypoints.size() - 1)
          {
            // Fix
            for (int i = 0; i < R; i++)
            {
              M(fixRow, fixCol) = 1;
              fixRow++;
              fixCol++;
            }
            // No Float          
          }
          else
          {
            // 1 Fix
            M(fixRow, fixCol) = 1;
            fixRow++;
            fixCol++;
            fixCol += R-1;
            // R-1 Float
            for (int i = 0; i < R-1; i++)
            {
              M(floatRow, floatCol) = 1;
              floatRow++;
              floatCol++;
            }
            floatRow -= R-1;    // Make sure end point and next start point has same derivative values
            floatCol += 1;        
          }
        }
        // OK, all bookeeping done, start the actual optimization...
        colvec po;        
        mat iA  = inv(A);
        if (waypoints.size() > 1)   // General case
        {
          mat R       = M * trans(iA) * Q * iA * trans(M);        
          mat Rfp     = R.submat(0,fixCnt, fixCnt-1, R.n_cols-1);
          mat Rpp     = R.submat(fixCnt, fixCnt, R.n_rows-1, R.n_cols-1);      
          colvec D    = M * b;
          colvec Df   = D.rows(0, fixCnt-1);
          colvec Dp   = -inv(Rpp) * trans(Rfp) * Df;
          colvec Dopt = join_cols(Df, Dp);
          po          = iA * trans(M) * Dopt;
        }
        else                        // One segment only
        {
          po = iA * b;
        }
//        if(axis == 0)
//          {cout<<"M: \n"<<M<<endl;
//           cout<<"b: \n"<<b<<endl;}
        /*cout<<"M * M'\n"<<M * trans(M)<<endl;
        cout<<"M' * M\n"<<trans(M) * M<<endl;*/
        // Store back to the data structure I want...
        for (unsigned int k = 0; k < waypoints.size(); k++)
        {
          if (axis == 0)
            polyx.push_back(po.rows(k*(N+1), (k+1)*(N+1)-1));
          else if (axis == 1)
            polyy.push_back(po.rows(k*(N+1), (k+1)*(N+1)-1));          
          else
            polyz.push_back(po.rows(k*(N+1), (k+1)*(N+1)-1));                    
        }
      }
     
      // Calculate coefficients for yaw      
      mat    Q = zeros<mat>((Nyaw+1)*waypoints.size() , (Nyaw+1)*waypoints.size());
      mat    A = zeros<mat>((Nyaw+1)*waypoints.size() , (Nyaw+1)*waypoints.size());
      mat    M = zeros<mat>(2*Ryaw+2*(waypoints.size()-1)+(Ryaw-1)*(waypoints.size()-1) , (Nyaw+1)*waypoints.size());
      colvec b = zeros<colvec>((Nyaw+1)*waypoints.size());
      // for Bookeeping M matrix...
      unsigned int fixCnt    = Ryaw                /*start, first seg*/          + Ryaw                 /*end, last seg*/         + 
                              (waypoints.size()-1) /* start, except first seg */ + (waypoints.size()-1) /* end, except last seg */;   
      unsigned int fixRow    = 0;   
      unsigned int fixCol    = 0;       // Position (0th dev) of start of first seg
      unsigned int floatRow  = fixCnt;  
      unsigned int floatCol  = Ryaw+1;  // Velocity (1th dev) of end   if first seg
      for (unsigned int k = 0; k < waypoints.size(); k++)
      {       
        // Start & End point, time
        double    yaw0  = (k == 0)?inityaw:waypoints[k-1](3);
        double    dyaw0 = (k == 0)?initdyaw:0.0;          
        double    yaw1  = waypoints[k](3);
        ros::Time t0    = (k == 0)?initt:ts[k-1];      
        ros::Time t1    = ts[k];
        double    dt    = (t1-t0).toSec();         
        // Construct cost function
        mat _Q = zeros<mat>(Nyaw+1, Nyaw+1);
        for (int i = 0; i <= Nyaw; i++)
        {
          for (int l = 0; l <= Nyaw; l++)
          {
            if (i >= Ryaw && l >= Ryaw)
            {
              _Q(i,l) = 2.0 * pow(dt, i+l-2*Ryaw+1) / (i+l-2*Ryaw+1);
              for (int m = 0; m <= Ryaw-1; m++)
                _Q(i,l) *= (i-m) * (l-m);
            }
          }
        }  
        // Construct constraints A
        mat _A0 = eye<mat>(Ryaw, Nyaw+1);
        for (int r = 0; r <= Ryaw-1; r++)
          for (int m = 0; m <= r-1; m++)
            _A0(r,r) *= r - m;
        
        mat _A1 = zeros<mat>(Ryaw, Nyaw+1);
        for (int r = 0; r <= Ryaw-1; r++)
        {
          for (int n = r; n <= Nyaw; n++)
          {
            _A1(r,n) = pow(dt, n-r);
            for (int m = 0; m <= r-1; m++)
              _A1(r,n) *= n - m;
          }
        }
        mat _A = join_cols(_A0, _A1);
        //Construct constraints b
        colvec _b = zeros<colvec>(Nyaw+1);
        _b(0)     = yaw0;
        _b(1)     = dyaw0; // Init yaw velocity of the start of the first segment
        _b(Ryaw)  = yaw1;
        // Construct full matrices    
        Q.submat(k*(Nyaw+1), k*(Nyaw+1), (k+1)*(Nyaw+1)-1, (k+1)*(Nyaw+1)-1) = _Q;
        A.submat(k*(Nyaw+1), k*(Nyaw+1), (k+1)*(Nyaw+1)-1, (k+1)*(Nyaw+1)-1) = _A;
        b.rows(k*(Nyaw+1), (k+1)*(Nyaw+1)-1)                                 = _b;          
        // Construct mapping matrix
        // Start Point
        if (k == 0)
        {
          // Fix
          for (int i = 0; i < Ryaw; i++)
          {
            M(fixRow, fixCol) = 1;
            fixRow++;
            fixCol++;
          }
          // No Float
        }
        else
        {
          // 1 Fix
          M(fixRow, fixCol) = 1;
          fixRow++;
          fixCol++;
          fixCol += Ryaw-1;
          // Ryaw-1 Float
          for (int i = 0; i < Ryaw-1; i++)
          {
            M(floatRow, floatCol) = 1;
            floatRow++;
            floatCol++;
          }
          floatCol += 1;
        }
        // End Point
        if (k == waypoints.size() - 1)
        {
          // Fix
          for (int i = 0; i < Ryaw; i++)
          {
            M(fixRow, fixCol) = 1;
            fixRow++;
            fixCol++;
          }
          // No Float          
        }
        else
        {
          // 1 Fix
          M(fixRow, fixCol) = 1;
          fixRow++;
          fixCol++;
          fixCol += Ryaw-1;
          // Ryaw-1 Float
          for (int i = 0; i < Ryaw-1; i++)
          {
            M(floatRow, floatCol) = 1;
            floatRow++;
            floatCol++;
          }
          floatRow -= Ryaw-1;    // Make sure end point and next start point has same derivative values
          floatCol += 1;        
        }
      }
      // OK, all bookeeping done, start the actual optimization...
      colvec po;        
      mat iA  = inv(A);
      if (waypoints.size() > 1)   // General case
      {
        mat R       = M * trans(iA) * Q * iA * trans(M);        
        mat Rfp     = R.submat(0,fixCnt, fixCnt-1, R.n_cols-1);
        mat Rpp     = R.submat(fixCnt, fixCnt, R.n_rows-1, R.n_cols-1);      
        colvec D    = M * b;
        colvec Df   = D.rows(0, fixCnt-1);
        colvec Dp   = -inv(Rpp) * trans(Rfp) * Df;
        colvec Dopt = join_cols(Df, Dp);
        po          = iA * trans(M) * Dopt;
      }
      else                        // One segment only
      {
        po = iA * b;
      }

      //ROS_BREAK();}
      // Store back to the data structure I want...
      for (unsigned int k = 0; k < waypoints.size(); k++)
        polyyaw.push_back(po.rows(k*(Nyaw+1), (k+1)*(Nyaw+1)-1));  
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

      // fill in trajectory id
      cmd.trajectory_id = trajectory_id;

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

      // Reach final goal
      if ((t - ts[idx]).toSec() >= 0)      
      {
        cmd.position.x = waypoints.back()(0);
        cmd.position.y = waypoints.back()(1);
        cmd.position.z = waypoints.back()(2);   
        cmd.yaw        = waypoints.back()(3);   

        // fill in trajectory flag
        cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
      }
      else
      {
        ros::Time t0 = (idx == 0)?initt:ts[idx-1];        
        double    dt = (t - t0).toSec();
        // XYZ, handle hover case
        if (ishoverxyz)
        {
          cmd.position.x = hoverwaypoint(0);
          cmd.position.y = hoverwaypoint(1);
          cmd.position.z = hoverwaypoint(2);   
        }
        else
        {
          colvec px    = polyx[idx];
          colvec py    = polyy[idx];
          colvec pz    = polyz[idx];
          colvec pt    = zeros<colvec>(N+1);
          for (int k = 0; k <= N; k++)
            pt(k) = pow(dt, k);        
          for (int k = 0; k <= N; k++)
          {
            cmd.position.x += px(k) * pt(k);
            cmd.position.y += py(k) * pt(k);
            cmd.position.z += pz(k) * pt(k);     
            if (k >= 1)
            {
              cmd.velocity.x += k * px(k) * pt(k-1);
              cmd.velocity.y += k * py(k) * pt(k-1);
              cmd.velocity.z += k * pz(k) * pt(k-1);                        
            }
            if (k >= 2)               
            {
              cmd.acceleration.x += k * (k-1) * px(k) * pt(k-2);
              cmd.acceleration.y += k * (k-1) * py(k) * pt(k-2);
              cmd.acceleration.z += k * (k-1) * pz(k) * pt(k-2);                        
            }
          }
        }

        // Yaw, handle hover case
        if (ishoveryaw)
        {
          cmd.yaw = hoverwaypoint(3);        
        }
        else
        {
          colvec pyaw  = polyyaw[idx];
          colvec pyawt = zeros<colvec>(Nyaw+1);
          for (int k = 0; k <= Nyaw; k++)
            pyawt(k) = pow(dt, k);        
          for (int k = 0; k <= Nyaw; k++)
          {
            cmd.yaw += pyaw(k) * pyawt(k);    
            if (k >= 1)
              cmd.yaw_dot += k * pyaw(k) * pyawt(k-1);                     
          }  
        }       
      
        // fill in trajectory flag
        cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
      }
    }
    
    ros::Time GetFinalTime()
    {
      return ts.back();
    }
    
    const vector<colvec>& GetWaypoints()
    {
      return waypoints;
    }    
    
    int GetCurrWaypointIdx(ros::Time t)
    {
      // Find current waypoint
      int idx = 0;
      for (; idx < ts.size(); idx++)
      {
        if ((ts[idx] - t).toSec() >= SEGMENT_SWITCH_T)
          break;
        if (idx == ts.size() - 1)
          break;
      }         
      return idx;       
    }

  private:
     
    static constexpr double HOVER_D           = 0.2;
    static constexpr double HOVER_YAW         = 5.0 * M_PI / 180;
    static constexpr double REUSE_T_RATIO     = 1.5;
    static constexpr double EPSILON_D         = 10e-5;
    static constexpr double SEGMENT_SWITCH_T  = 0.5;    
    colvec            initp;      // Initial position
    colvec            initv;      // Initial velocity    
    colvec            inita;      // Initial acceleration        
    double            inityaw;    // Initial yaw
    double            initdyaw;   // Initial yaw velocity
    ros::Time         initt;      // Initial time
    vector<colvec>    waypoints;  // Waypoints
    vector<ros::Time> ts;         // Time to reach each waypoint
    double            vel;        // Maximum speed
    double            acc;        // Maximum acceleration
    double            vel_yaw;    // Maximum yaw velocity
    int               R;          // Order of derivative to be minimized        
    int               N;          // Polynomial order
    int               Ryaw;       // Order of derivative to be minimized for yaw
    int               Nyaw;       // Polynomial order for yaw
    vector<colvec>    polyx;      // Polynomial coefficients X   
    vector<colvec>    polyy;      // Polynomial coefficients Y       
    vector<colvec>    polyz;      // Polynomial coefficients Z           
    vector<colvec>    polyyaw;    // Polynomial coefficients Yaw   
    bool              ishoverxyz;
    bool              ishoveryaw;    
    colvec            hoverwaypoint;
    unsigned int trajectory_id;
};

#endif
