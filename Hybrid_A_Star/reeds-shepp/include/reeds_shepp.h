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
          float t=std::numeric_limits<float>::max(), 
          float u=0., float v=0.,
          float w=0., float x=0.);
        
        float length() const { return totalLength_; }

        /** Path segment types */
        const ReedsSheppPathSegmentType* type_;
        /** Path segment lengths */
        float length_[5];
        /** Total length */
        float totalLength_;
    };

    typedef vector<vector<float>> sample_paths;
    typedef vector<ReedsSheppPathSegmentType> path_types;

    ReedsSheppStateSpace(float turningRadius) : rho_(turningRadius) {}

    float distance(float q0[3], float q1[3]);
    path_types type(float q0[3], float q1[3]);
    sample_paths sample(float q0[3], float q1[3], float step_size);

    /** \brief Return the shortest Reeds-Shepp path from 
     *   SE(2) state state1 to SE(2) state state2 */
    ReedsSheppPath reedsShepp(float q0[3], float q1[3]);

protected:
    void interpolate(float q0[3], ReedsSheppPath &path, 
                     float seg, vector<float>& s);

    /** \brief Turning radius */
    float rho_;
};

#endif