#ifndef IMUMODEL_IMUMODEL_HPP__
#define IMUMODEL_IMUMODEL_HPP__

#include <base/eigen.h>
#include <base/samples/imu.h>
#include <boost/random.hpp>

namespace imumodel
{

struct Configuration
{
    /** Random seed to be used for random number generation. Use -1 to
     * initialize seed with current time. */
    int seed;
    /** Frequency of the sampling time of the imu values. */
    double dt;

    /** Deterministic error matrix containing scale factor and misalignement */
    base::Matrix3d Dacc;
    /** Deterministic error matrix containing scale factor and misalignement */
    base::Matrix3d Dgyro;

    /** velocity random walk for accelerometers (m/s/sqrt(s)) */
    base::Vector3d accrw;
    /** acceleration random walk for accelerometers (m/s^2/sqrt(s)) */
    base::Vector3d accrrw;
    /** bias instability for accelerometers (m/s^2) */
    base::Vector3d accbias;
    /** is the reciprocal correlated noise for the bias instability
     * approximation by first Guass-Markov process and need to be determinated
     */
    base::Vector3d abeta;

    /** angle random walk for gyroscopes (rad/sqrt(s)) */
    base::Vector3d gyrorw;
    /** rate random walk for gyroscopes (rad/s/sqrt(s)) */
    base::Vector3d gyrorrw;
    /** bias instability for gyroscopes (rad/s) */
    base::Vector3d gyrobias;
    /** is the reciprocal correlated noise (1/sec) for the bias instability
     * approximation by first Guass-Markov process and need to be determinated
     */
    base::Vector3d gbeta;

    /** default configuration is error free */
    Configuration() :
	seed(-1),
	dt(0.01),
	Dacc( base::Matrix3d::Zero() ),
	Dgyro( base::Matrix3d::Zero() ),
	accrw( base::Vector3d::Zero() ),
	accrrw( base::Vector3d::Zero() ),
	accbias( base::Vector3d::Zero() ),
	abeta( base::Vector3d::Zero() ),
	gyrorw( base::Vector3d::Zero() ),
	gyrorrw( base::Vector3d::Zero() ),
	gyrobias( base::Vector3d::Zero() ),
	gbeta( base::Vector3d::Zero() ) 
    {}
};

class ImuError
{
public:
    static const int NUMAXIS = 3;

    void init();
    void reset();
    void step();
    void addNoise( base::samples::IMUSensors &imu_sample );
    void setConfiguration( const Configuration& config );
    const Configuration& getConfiguration() const;
    Configuration& getConfiguration();

    base::Vector3d getGyroError() const;

protected:
    Configuration config;

    base::Vector2d xax; /**< State vector for acc x axis model*/
    base::Vector2d xay; /**< State vector for acc y axis model*/
    base::Vector2d xaz; /**< State vector for acc z axis model*/

    base::Matrix2d Aax; /**< State Matrix for Accelerometer x axis */
    base::Vector2d Gax; /**< Input Matrix for Accelerometer x axis */
    base::Matrix2d Aay; /**< State Matrix for Accelerometer y axis */
    base::Vector2d Gay; /**< Input Matrix for Accelerometer y axis */
    base::Matrix2d Aaz; /**< State Matrix for Accelerometer z axis */
    base::Vector2d Gaz; /**< Input Matrix for Accelerometer z axis */
    base::Vector2d Ha; /**< Observation Matrix for Accelerometers */

    base::Vector2d xgx; /**< State vector for gyro x axis model*/
    base::Vector2d xgy; /**< State vector for gyro y axis model*/
    base::Vector2d xgz; /**< State vector for gyro z axis model*/

    base::Matrix2d Agx; /**< State Matrix for Gyro x axis */
    base::Vector2d Ggx; /**< Input Matrix for Gyro x axis */
    base::Matrix2d Agy; /**< State Matrix for Gyro y axis */
    base::Vector2d Ggy; /**< Input Matrix for Gyro y axis */
    base::Matrix2d Agz; /**< State Matrix for Gyro z axis */
    base::Vector2d Ggz; /**< Input Matrix for Gyro z axis */
    base::Vector2d Hg; /**< Observation Matrix for Gyroscopes */

    Eigen::Matrix <double,NUMAXIS,1> acc, gyros; /**< Accelerometer and gyroscopes output vectors */
    Eigen::Matrix <double,NUMAXIS,NUMAXIS> Dacc, Dgyro; /**< Deterministics error matrices for acc and gyros*/

    typedef boost::mt19937 RandomGenerator;
    RandomGenerator rng; 
    double GetNormalDistri(double mean, double sigma);
};

/** 
 * Generate Error for an AHRS system
 */
class AhrsError
{
    base::Transform3d orientationError;
    ImuError model;

public:
    AhrsError()
    {
	reset();
    }

    void setZGyroProperties( double bias, double rw, double rrw )
    {
	Configuration &config( model.getConfiguration() );
	config.gyrobias = base::Vector3d::UnitZ() * bias;
	config.gyrorw = base::Vector3d::UnitZ() * rw;
	config.gyrorrw = base::Vector3d::UnitZ() * rrw;
	model.init();
    };

    void reset( int seed = -1 )
    {
	model.getConfiguration().seed = seed;
	model.init();
	model.reset();
	orientationError = base::Transform3d::Identity();
    }

    void step()
    {
	model.step();
	base::Vector3d error = model.getGyroError();
	orientationError = Eigen::AngleAxisd( error.z(), Eigen::Vector3d::UnitZ() ) * 
	    orientationError;
    }

    base::Transform3d getErrorTransform()
    {
	return orientationError;
    }
};

}

#endif
