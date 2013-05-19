#include <imumodel/ImuError.hpp>
#include <base/pose.h>

using namespace imumodel;

int main( int argc, char** argv )
{
    AhrsError error;
    error.setZGyroProperties( 0, 6e-4, 3e-6 );
    for( int i=0; i<10000; i++ )
    {
	error.step();
	std::cout << base::getYaw( Eigen::Quaterniond( error.getErrorTransform().linear() ) ) << std::endl;
    }
}
