#include "ImuError.hpp"

using namespace imumodel;

void ImuError::init()
{
   // initialize random number generator
   unsigned long seed =  
	(config.seed == -1) ? time(0) : config.seed;
   rng = RandomGenerator( seed ); 

   base::Vector3d abeta = config.abeta;
   base::Vector3d accrrw = config.accrrw;
   base::Vector3d accbias = config.accbias;
   base::Vector3d gbeta = config.gbeta;
   base::Vector3d gyrorrw = config.gyrorrw;
   base::Vector3d gyrobias = config.gyrobias;
   double dt = config.dt;
    
   /** Fill the matrices with the configure values for the accelerometer model X axis **/
   Aax(0,0) = 1.0;
   Aax(0,1) = 0.0;
   Aax(1,0) = dt;
   Aax(1,1) = 1-(abeta[0]*dt);

   Gax (0,0) = accrrw[0]* abeta[0];
   Gax (1,0) = (accrrw[0]*sqrt(dt/3))+ abeta[0]*accbias[0];

   /** Fill the matrices with the configure values for the accelerometer model Y axis **/
   Aay(0,0) = 1.0;
   Aay(0,1) = 0.0;
   Aay(1,0) = dt;
   Aay(1,1) = 1-(config.abeta(1)*config.dt);

   Gay (0,0) = accrrw[1]* config.abeta(1);
   Gay (1,0) = (accrrw[1]*sqrt(dt/3))+ abeta[1]*accbias[1];

   /** Fill the matrices with the configure values for the accelerometer model Z axis **/
   Aaz(0,0) = 1.0;
   Aaz(0,1) = 0.0;
   Aaz(1,0) = dt;
   Aaz(1,1) = 1-(config.abeta(2)*config.dt);

   Gaz (0,0) = accrrw[2]* config.abeta(1);
   Gaz (1,0) = (accrrw[2]*sqrt(dt/3))+ abeta[2]*accbias[2];

   /** Measurement matrix **/
   Ha << 0,1;

   /** Fill the matrices with the configure values for the gyroscope model X axis **/
   Agx(0,0) = 1.0;
   Agx(0,1) = 0.0;
   Agx(1,0) = dt;
   Agx(1,1) = 1.0-(gbeta[0]*dt);

   Ggx (0,0) = gyrorrw[0]* gbeta[0];
   Ggx (1,0) = (gyrorrw[0]*sqrt(dt/3)) + gbeta[0]*gyrobias[0];

   /** Fill the matrices with the configure values for the gyroscope model Y axis **/
   Agy(0,0) = 1.0;
   Agy(0,1) = 0.0;
   Agy(1,0) = dt;
   Agy(1,1) = 1-(gbeta[1]*dt);

   Ggy (0,0) = gyrorrw[1]* gbeta[1];
   Ggy (1,0) = (gyrorrw[1]*sqrt(dt/3)) + gbeta[1]*gyrobias[1];

   /** Fill the matrices with the configure values for the gyroscope model Z axis **/
   Agz(0,0) = 1.0;
   Agz(0,1) = 0.0;
   Agz(1,0) = dt;
   Agz(1,1) = 1-(gbeta[2]*dt);

   Ggz (0,0) = gyrorrw[2]* gbeta[1];
   Ggz (1,0) = (gyrorrw[2]*sqrt(dt/3)) + gbeta[2]*gyrobias[2];

   /** Measurement matrix **/
   Hg << 0,1;

   /** deterministic matrices to zero **/
   Dacc = Eigen::Matrix<double,NUMAXIS,NUMAXIS>::Zero();
   Dgyro = Eigen::Matrix<double,NUMAXIS,NUMAXIS>::Zero();
}

void ImuError::reset()
{
    /** Reset the state vectors and the output vectors **/
    xax << 0.0,0.0;
    xay << 0.0,0.0;
    xaz << 0.0,0.0;
    
    xgx << 0.0,0.0;
    xgy << 0.0,0.0;
    xgz << 0.0,0.0;
    
    acc = Eigen::Matrix<double,NUMAXIS,1>::Zero();
    gyros = Eigen::Matrix<double,NUMAXIS,1>::Zero();
    
    Dacc = config.Dacc;
    Dgyro = config.Dgyro;
}

void ImuError::step()
{
    /** White noise for the Velocty Random Walk for accelerometers **/
    base::Vector3d stdacc;
    stdacc[0] = config.accrw[0]/sqrt(config.dt);
    stdacc[1] = config.accrw[1]/sqrt(config.dt);
    stdacc[2] = config.accrw[2]/sqrt(config.dt);
  
    /** Perform the model step for Accelerometers Xaxis **/
    xax = Aax * xax + Gax * GetNormalDistri(0,1.0);
    acc[0] = GetNormalDistri(0,stdacc[0]) + (Ha.transpose() * xax); // White noise + stochastic noise
    
    /** Perform the model step for Accelerometers Yaxis **/
    xay = Aay * xay + Gay * GetNormalDistri(0,1.0);
    acc[1] = GetNormalDistri(0,stdacc[1]) + (Ha.transpose() * xay); // White noise + stochastic noise
    
    /** Perform the model step for Accelerometers Zaxis **/
    xaz = Aaz * xaz + Gaz * GetNormalDistri(0,1.0);
    acc[2] = GetNormalDistri(0,stdacc[2]) + (Ha.transpose() * xaz); // White noise + stochastic noise
    
    /** White noise for the Angular Random Walk for gyroscopes **/
    base::Vector3d stdgyro;
    stdgyro[0] = config.gyrorw[0]/sqrt(config.dt);
    stdgyro[1] = config.gyrorw[1]/sqrt(config.dt);
    stdgyro[2] = config.gyrorw[2]/sqrt(config.dt);
    
    /** Perform the model step for gyroscopes Xaxis **/
    xgx = Agx * xgx + Ggx * GetNormalDistri(0,1.0);
    gyros[0] = GetNormalDistri(0, stdgyro[0]) + (Hg.transpose() * xgx); // White noise + stochastic noise
    
    /** Perform the model step for gyroscopes Yaxis **/
    xgy = Agy * xgy + Ggy * GetNormalDistri(0,1.0);
    gyros[1] = GetNormalDistri(0, stdgyro[1]) + (Hg.transpose() * xgy); // White noise + stochastic noise
    
    /** Perform the model step for gyroscopes Zaxis **/
    xgz = Agz * xgz + Ggz * GetNormalDistri(0,1.0);
    gyros[2] = GetNormalDistri(0, stdgyro[2]) + (Hg.transpose() * xgz); // White noise + stochastic noise
}

void ImuError::addNoise( base::samples::IMUSensors &imu_sample )
{
    /** Include the deterministic error to the model (accelerometer and gyroscopes) **/
    imu_sample.acc = (Eigen::Matrix<double,NUMAXIS,NUMAXIS>::Identity() + Dacc) * imu_sample.acc + acc;
    imu_sample.gyro = (Eigen::Matrix<double,NUMAXIS,NUMAXIS>::Identity() + Dgyro) * imu_sample.gyro + gyros;
}

double ImuError::GetNormalDistri(double mean, double sigma)
{
    typedef boost::normal_distribution<double> NormalDistribution;
    typedef boost::variate_generator<RandomGenerator&,NormalDistribution> GaussianGenerator;

    /* Choose Normal Distribution */
    NormalDistribution gaussian_dist(mean, sigma);

    /* Create a Gaussian Random Number generator
     *  by binding with previously defined
     *  normal distribution object
     */
    GaussianGenerator generator(rng, gaussian_dist);

    // sample from the distribution
    return generator();
}

void ImuError::setConfiguration( const Configuration& config )
{
    this->config = config;
}
    
const Configuration& ImuError::getConfiguration() const
{
    return config;
}

