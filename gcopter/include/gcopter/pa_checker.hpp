# include "trajectory.hpp"
# include <Eigen/Eigen>


namespace pa_checker
{
    class Pa_checker
    {
    private:
        double progress_t;
        double a_max;
        double max_dist;
        bool safe;
        double angle;


    public:
        Pa_checker(double progress, double alpha, double amax, double maxdist, bool safeFlag)
        {
            progress_t = progress;
            angle = alpha;
            a_max = amax;
            max_dist = maxdist;
            safe = safeFlag;

        }

        inline void check(Trajectory<5> traj, Eigen::Vector4d quat, Eigen::Vector3d pos, double speed, double delta)
        {
            if(progress_t <= delta) progress_t = delta;

            Eigen::Quaterniond q(quat(0),quat(1),quat(2),quat(3));
            Eigen::Vector3d h = q*Eigen::Vector3d::UnitX();
            h = h.normalized();

            for (double t = progress_t+0.01; t<= traj.getTotalDuration(); t += 0.05)
            {
                Eigen::Vector3d check_pos = traj.getPos(t) - pos;
                Eigen::Vector3d unit_check = check_pos.normalized();

                if((h.dot(unit_check) > cos(angle/2) ) && (h.dot(check_pos) <= max_dist) )
                {
                    progress_t = t;
                    continue;
                }
                else
                {
                    double s = (traj.getPos(progress_t)-pos).norm();
                    safe = (speed*speed - 2* a_max * s )> 0 ? false : true;
                    //std::cout<< safe << " " << progress_t  <<" " << speed << " " << a_max << " " << s  << std::endl;
                    break;
                    

                }

            }
            return;


        }

        inline double getProgress()
        {
            return progress_t;
        }

        inline bool getSafeFlag()
        {
            return safe;
        }

        inline void clear()
        {
            progress_t = 0.0;
            safe = false;
        }

    };
    


}