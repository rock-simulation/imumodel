#include <imumodel/ImuError.hpp>
#include <base/pose.h>
#include <boost/lexical_cast.hpp>

using namespace imumodel;

int main( int argc, char** argv )
{
    int count = 10;
    int seed = -1;
    if( argc > 1 )
	count = boost::lexical_cast<int>(argv[1]);
    if( argc > 2 )
	seed = boost::lexical_cast<int>(argv[2]);

    AhrsError error;
    error.setZGyroProperties( 0, 6e-4, 3e-6 );
    error.reset( seed );
    for( int i=0; i<count; i++ )
    {
	error.step();
	std::cout << base::getYaw( Eigen::Quaterniond( error.getErrorTransform().linear() ) ) << std::endl;
    }
}
