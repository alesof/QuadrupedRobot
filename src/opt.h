#include "ros/ros.h"
#include "Eigen/Dense"
#include "alglib/optimization.h"

class OPT {
    public:
        OPT(int control_variables, int stance_constraint, int swing_constraint);
        void setQ( Eigen::MatrixXd Q_ );
        void setc( Eigen::VectorXd c_ );
        void setL_swing( Eigen::MatrixXd L_swing ,int n_lt);
        void setL_stance( Eigen::MatrixXd L_stance );
        void set_Lt_stance( Eigen::VectorXd Lt_stance );
        void set_Lt_swing( Eigen::VectorXd Lt_swing );
        void opt_stance( Eigen::VectorXd & x_ );
        void opt_swing( Eigen::VectorXd & x_ );
        //SET _L_swing
        //SET _L_stance


        //GET tau_swing
        //Get tau_stanc
    private:
        int _control_variables;
        int _stance_constraint;
        int _swing_constraint;
        
        alglib::real_2d_array _Q, _R;
        alglib::real_2d_array _L_swing, _L_stance;
        alglib::real_1d_array _c;
        alglib::integer_1d_array _Lt_swing, _Lt_stance;
        alglib::real_1d_array _x;




};


