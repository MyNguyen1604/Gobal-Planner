#include <iostream>
#include <math.h>
#include <set>
#include <algorithm>
#include <iterator>

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

using namespace std;
using std::string;

#ifndef GLOBALPLANNER_CPP
#define GLOBALPLANNER_CPP

#define PI 3.14159265
typedef struct {
    int index;
    float fcost;
} cell_;
struct cmp{
    bool operator()(cell_ a, cell_ b) {return a.fcost < b.fcost;}
};
namespace globalplanner{
    class GlobalPlanner : public nav_core::BaseGlobalPlanner{
        public:
        GlobalPlanner();
        ~GlobalPlanner();
        GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        
        float calculateHcost(int Cell, int goalCell);
        void addneighbortoOpen(multiset<cell_, cmp>& OPEN, int neighborCell, int goalCell, float gcost[]);
        float getMoveCost(int startCell, int neighborCell);
        
        vector<int> planner(int startCell, int goalCell);
        vector<int> constructPath(int startCell, int goalCell, float gcost[]);
        vector<int> reconstructPath(vector<int> path_);
        bool freeZone(int cell1, int cell2);
        
        vector<int> findpath(int startCell, int goalCell, float gcost[]);
        vector<int> findNeighborCell(int currentIndex);
        
        
        int converttoCellIndex(float x, float y);
        int getCellIndex(int i, int j);
        int getRowIndex(int index);
        int getColIndex(int index);
        void getCoordinate(float& x, float& y);
        
        void converttoMap(int index, float& x, float& y);
        
        
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
        bool iscellInsideMap(float x, float y);
        bool isFreeCell(int index);
        bool isStartAndGoalCellsValid(int startCell, int goalCell);
        
        
        bool initialized_;
        costmap_2d::Costmap2DROS* costmap_ros_;
        costmap_2d::Costmap2D* costmap_;
        float originX, originY;
        float reso; //resolution of costmap
        int width;
        int height;
        int mapsize;
        
        //float tBreak;
        //Description of system
        float lr;
        float lt;
        float phi_max;
        float d;
        //float K = 1.05;//safe gain
        
        float xr;
        float xa;//xa = 1/r_truck_min
        //Function ralative with truck-trailer system
        void computeES(void);
        
    }; 

};
#endif
