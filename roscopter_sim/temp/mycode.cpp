    // #######################################

    actual_forces_.Fx = -1.0*linear_mu_*ur;
    actual_forces_.Fy = -1.0*linear_mu_*vr;
    actual_forces_.Fz = -1.0*linear_mu_*wr - applied_forces_.Fz - ground_effect;
    actual_forces_.l = -1.0*angular_mu_*p + applied_forces_.l;
    actual_forces_.m = -1.0*angular_mu_*q + applied_forces_.m;
    actual_forces_.n = -1.0*angular_mu_*r + applied_forces_.n;

    // Eigen::Matrix<double, 3,3> R_inv;
    // R_inv = R.inverse();

    Eigen:: Matrix<double, 3,1> F_ii;
    F_ii << actual_forces_.Fx, -actual_forces_.Fy, -actual_forces_.Fz;
    Eigen::MatrixXd F_b;
    // F_b = R_inv*F_ii;
    F_b = R.inverse()*F_ii;
//    std::cout<<"F_x: "<<F_b(0,0)<< " F_y: "<<F_b(1,0)<<" F_z: "<<F_b(2,0)<<std::endl;

    Eigen:: Matrix<double, 3,1> M_ii;
    M_ii<< actual_forces_.l, -actual_forces_.m, -actual_forces_.n;
    Eigen::MatrixXd M_b;
    M_b = R.inverse()*M_ii;
    // std::cout<<M_b<<std::endl; 

    Eigen:: Matrix<double, 3,1> body_vector;
    body_vector<< 0, 0, 1;
    Eigen::MatrixXd b_v_i;
    b_v_i = R*body_vector;

    double beta = acos(  -b_v_i(0,0)/sqrt(pow(b_v_i(0,0),2) + pow(b_v_i(1,0),2) + pow(b_v_i(2,0),2))   );

    double sigma_T = F_b(0,0) - (-mass_*9.81*c(beta));

    Eigen::MatrixXd omega_square;
    // Eigen::Matrix<double 4,1> omega;
    Eigen::Matrix<double, 4,1> FT_bb;
    FT_bb<< sigma_T, M_b(0,0), M_b(1,0), M_b(2,0);
    omega_square =coeff.inverse()*FT_bb;
    std::cout<<omega_square<<std::endl;
    // omega << sqrt(omega_square(0,0)), sqrt(omega_square(1,0)), sqrt(omega_square(2,0)), sqrt(omega_square(3,0));
    // std::cout<<"W_1: "<<omega(0,0)<< " W_2: "<<omega(1,0)<<" W_3: "<<omega(2,0)<<"W_4"<<omega(3,0)<<std::endl;

    //##################################################################

    MY CODE GOES HERE    
    Eigen::Matrix<double, 4,4> coeff;
    double Ct = 0.00028;
    double dCt = 0.000012;
    double Cq = 0.000012;
    coeff << Ct,Ct, Ct, Ct,
            0, dCt, 0, -dCt,
            -dCt, 0, dCt, 0,
            -Cq, Cq, -Cq, Cq;

    Eigen::MatrixXd angular_speeds;
    Eigen::Matrix<double, 4,1> FT_b;
    FT_b << applied_forces_.Fz, applied_forces_.l, applied_forces_.m, applied_forces_.n;
    angular_speeds = coeff.inverse()*FT_b;
    std::cout<<"Om_1: "<<angular_speeds(0,0)<<"Om_2: "<<angular_speeds(1,0)<<"Om_3: "<<angular_speeds(2,0)<<"Om_4: "<<angular_speeds(3,0)<<std::endl;

    Eigen::Matrix<double, 3,3> R;
    R<< (c(-psi)*c(-theta)-s(phi)*s(-psi)*s(-theta)), -c(phi)*s(phi), (c(-psi)*s(-theta)+c(-theta)*s(phi)*s(-psi)),
            (c(-theta)*s(-psi)+c(-psi)*s(phi)*s(-theta)), c(phi)*c(-psi), (s(-psi)*s(-theta) - c(-psi)*c(-theta)*s(-psi)),
            (-c(phi)*s(-theta)), s(phi), c(phi)*c(-theta);    

   //#############################################

    // MY CODE GOES HERE   

    Eigen::Matrix<double, 4,4> coeff;
    double Ct = 0.00028;
    double dCt = 0.000012;
    double Cq = 0.000012;
    coeff << Ct,Ct, Ct, Ct,
            0, dCt, 0, -dCt,
            -dCt, 0, dCt, 0,
            -Cq, Cq, -Cq, Cq;

    Eigen::Matrix<double, 3,3> R;
    R<< (c(psi)*c(theta)-s(phi)*s(psi)*s(theta)), -c(psi)*c(theta), (c(psi)*s(theta)+c(theta)*s(phi)*s(psi)),
            (c(theta)*s(psi)+c(psi)*s(phi)*s(theta)), c(phi)*c(psi), (s(psi)*s(theta) - c(psi)*c(theta)*s(psi)),
            (-c(phi)*s(theta)), s(phi), c(phi)*c(theta);

    Eigen::MatrixXd FT_b = coeff * angles;

//    std::cout << FT_b(2,0) <<std::endl;
    Eigen:: Matrix<double, 3,1> F;
    F << 0, 0, FT_b(0,0);

    Eigen:: Matrix<double, 3,1> M;
    M << FT_b(1,0), FT_b(2,0), FT_b(3,0);

    Eigen::MatrixXd F_i;
    F_i = R * F;

    Eigen::MatrixXd M_i;
    M_i = R * M;

    std::cout<< F_i(2,0) <<std::endl;
    // std::cout<< M_i <<std::endl;

    // actual_forces_.Fx = F_i(0,0);
    // actual_forces_.Fy = F_i(1,0);
    // actual_forces_.Fz = F_i(2,0);
    // actual_forces_.l = M_i(0,0);
    // actual_forces_.m = M_i(1,0);
    // actual_forces_.n = M_i(2,0);

    
  //  MY CODE GOES HERE