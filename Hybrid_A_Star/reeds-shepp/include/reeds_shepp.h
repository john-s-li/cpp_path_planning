#ifndef SPACES_REEDS_SHEPP_STATE_SPACE_
#define SPACES_REEDS_SHEPP_STATE_SPACE_

#include <tuple>
#include <vector> 

#include <cassert>

using namespace std;

class ReedsSheppPath;

class ReedsSheppStateSpace
{
  public:
    enum ReedsSheppPathSegmentType { 
      RS_NOP=0, RS_LEFT=1, RS_STRAIGHT=2, RS_RIGHT=3 };
    
    /** \brief Reeds-Shepp path types */
    static const ReedsSheppPathSegmentType reedsSheppPathType[18][5];
    
    /** \brief Complete description of a ReedsShepp path */
    class ReedsSheppPath
    {
      public:
        ReedsSheppPath(
          const ReedsSheppPathSegmentType* type=reedsSheppPathType[0],
          double t=std::numeric_limits<double>::max(), 
          double u=0., double v=0.,
          double w=0., double x=0.);
        
        double length() const { return totalLength_; }

        /** Path segment types */
        const ReedsSheppPathSegmentType* type_;
        /** Path segment lengths */
        double length_[5];
        /** Total length */
        double totalLength_;
    };

    typedef vector<vector<double>> sample_paths;
    typedef vector<ReedsSheppPathSegmentType> path_types;

    ReedsSheppStateSpace(double turningRadius) : rho_(turningRadius) {}

    double distance(double q0[3], double q1[3]);
    path_types type(double q0[3], double q1[3]);
    sample_paths sample(double q0[3], double q1[3], double step_size);

    /** \brief Return the shortest Reeds-Shepp path from 
     *   SE(2) state state1 to SE(2) state state2 */
    ReedsSheppPath reedsShepp(double q0[3], double q1[3]);

protected:
    void interpolate(double q0[3], ReedsSheppPath &path, 
                     double seg, vector<double>& s);

    /** \brief Turning radius */
    double rho_;
};

#endif