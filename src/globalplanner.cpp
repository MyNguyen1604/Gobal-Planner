#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include "globalplanner.h"


/*move_base can thoi gian de co the khoi tao cac node*/
//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(globalplanner::GlobalPlanner, nav_core::BaseGlobalPlanner)


/**** Initialize Variables ***/
/*****************************/
bool* OGM;
static const float INFINIT_COST = INT_MAX; //maximun of int type
float infinity = numeric_limits<float>::infinity(); // infinity of float type

int clock_gettime(clockid_t clk_id, struct timespect *tp);

timespec diff(timespec start, timespec end)
{
  timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}

/*********** Class Namspace *************/
/****************************************/
namespace globalplanner{
     /**** Constructor of Class GlobalPlanner****/
     /*******************************************/
     GlobalPlanner::GlobalPlanner():costmap_ros_(NULL), initialized_(false){}
     GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
          initialize(name, costmap_ros);
     }
     /****** Destructor of Class GlobalPlanner******/
     /**********************************************/
     GlobalPlanner::~GlobalPlanner(){
          delete[] OGM;
     }
     
     
     /******* Initialized Function *******/
     /************************************/
     void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
          if(!initialized_){
              costmap_ros_ = costmap_ros;
              costmap_ = costmap_ros_->getCostmap();
              
              ros::NodeHandle private_nh("~/" + name);
              
              computeES();
              //Detail of costmap
              originX = costmap_->getOriginX();
              originY = costmap_->getOriginY();
              
              width = costmap_->getSizeInCellsX();
              height = costmap_->getSizeInCellsY();
              mapsize = width*height;
              reso = costmap_->getResolution();
              //tBreak = 1 + 1/(width + height);
              
              OGM = new bool [mapsize];
             
              for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++){
                 for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++){
                    unsigned int cost = static_cast<unsigned int>(costmap_->getCost(ix,iy));//da bao gom kich thuoc inlation_radius
                    if (cost == 0){ //Free cell in costmap
                       OGM[iy*width + ix] = true;
                    }
                    else{
                       OGM[iy*width + ix] = false;
                    }
                 }
              }
              //reOGM = OGM;
              //Off-set path three cell away from obstacle inlating
              
              /*for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++){
                 for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++){
                 
                     if (!(OGM[iy*width + ix])){
                     
                         for(int i = -1; i < 2; i++){
                            for (int j = -1; j < 2; j++){
                            
                                if ( (ix + i > 0) && (ix + i < width) && (iy + j > 0) && (iy + j < height) && !( (i == 0) && (j==0) ) ){
                                
                                   reOGM[(iy + j)*width + (ix+i)] = false;
                                }//end if
                                
                            }//end for
                         }//end for
                         
                     }//end if
                     clea
                 }//end for
              }//end for
              OGM = reOGM;
              
              delete[] reOGM;*/
              
              initialized_ = true;
                 
              //cout << "Detail of costmap: "<<" Weight: "<<width<<", Height: "<<height<<", Resolution: "<<reso;
              
          }
          else
              ROS_WARN("This planner has already been initialized... doing nothing");
     }
     /************************* Computing ES of truck - trailer system*****************************/
     void GlobalPlanner::computeES(void){
          cout << "========Description of system========"<<endl;
          cout << "Distance from center of truck to joint, lr(m): " <<endl;
          cin >> lr;
          cout << "Distance from joint to center of trailer, lt(m): "<<endl;
          cin >> lt;
          cout << "Maximum width of truck and trailer, d: "<<endl;
          cin >> d;
          cout <<"The maximum angle of joint, phi_max(degree): "<<endl;
          cin >> phi_max;
          cout << "=======Initialize Complete======="<<endl;
          cout <<"================================="<<endl;
          float rmin;//minimum turning radius of truck
          float rmin2;//minimum turning radius of trailer

          phi_max = phi_max * PI/180.0;
            
          if (lt >= lr){
              rmin2 = (lr + lt*cos(phi_max))/(sin(phi_max));
              rmin = sqrt( pow(rmin2,2.0) + 2*( pow(lt,2.0) - pow(lr,2.0) ));
              xr = rmin - rmin2 + d/2;
          }
          else{
              rmin = (lr + lt*cos(phi_max))/(sin(phi_max));
              rmin2 = sqrt( pow(rmin,2.0) + 2*( pow(lr,2.0) - pow(lt,2.0) ));
              xr = rmin2 - rmin + d/2;
           }
           xa = 1/rmin;
           cout << "Minimum turning of truck: "<<rmin<<endl;
           cout << "Minimum turning curve of truck: "<< xa <<endl;
           cout << "Equivalent size of truck-trailer: "<<xr<<endl;
          
     }
        
     
     
     /******************** makePlan function / service makePlan is called by move_base ******************/
     /*********************************************************/
     
     bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
          
          
          if (!initialized_)
          {
             ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
             return false;
          }
          if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
             ROS_ERROR("The planner will only accept goal in %s frame, but goal is in %s frame", costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
             return false;
          }
          
          
          
          
          
          ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
          plan.clear();

          float startX = start.pose.position.x;
          float startY = start.pose.position.y;
          
          float goalX = goal.pose.position.x;
          float goalY = goal.pose.position.y;
          
          //Convert start and goal to coordinate of costmap
          getCoordinate(startX, startY);
          getCoordinate(goalX, goalY);
          
          //cout << "Coordinate of start("<<startX<<", "<<startY<<")"<<endl;
          //cout << "Coordinate of goal("<<goalX<<", "<<goalY<<")"<<endl;
          
          //Convert to costmap cell
          int startCell;
          int goalCell;
          if (iscellInsideMap(startX, startY)&&iscellInsideMap(goalX, goalY)){
              //Considering startCell and goalCell index in OGM[]
              startCell = converttoCellIndex(startX, startY);
              goalCell = converttoCellIndex(goalX, goalY);
          }
          else{
              ROS_ERROR("Start or Goal is out of map");
              return false;
          }
          if(!isStartAndGoalCellsValid(startCell, goalCell)){
              ROS_ERROR("Start and Goal is not valid... Please try again!");
              return false;
          }
          //cout << "Cell index of start: "<<startCell<<endl;
          //cout << "Cell index of goal: "<<goalCell<<endl;
          vector<int> path;
          
          path.clear();
          path = planner(startCell, goalCell);
          
          //cout << path.size() << endl;
          if (path.size() > 0){
              float x = 0.0;
              float y = 0.0;
              int index;
              geometry_msgs::PoseStamped pose = goal;
              for (unsigned int i = 0; i < path.size() - 1; i++){
                  index = path[i];
                  converttoMap(index, x, y);
                  
                  pose.pose.position.x = x;
                  pose.pose.position.y = y;
                  pose.pose.position.z = 0.0;
                  
                  pose.pose.orientation.x = 0.0;
                  pose.pose.orientation.y = 0.0;
                  pose.pose.orientation.z = 0.0;
                  pose.pose.orientation.w = 1.0;
                  plan.push_back(pose);
              }//end for
              plan.push_back(goal);
          }
          else{
              ROS_ERROR("This plan can't find path.... Please try again!");
              return false;
          }
          
          float path_length = 0.0;
          vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();
          geometry_msgs::PoseStamped last_pose;
          last_pose = *it;
          it++;
          for(; it!=plan.end(); it++){
              path_length += hypot((*it).pose.position.x - last_pose.pose.position.x, (*it).pose.position.y - last_pose.pose.position.y);
              last_pose = *it;
          
          }
          cout << "Path length: "<<path_length<<" (m)"<<endl;
          
          return true;
     }
     
     
          //check startCell and goalCell is valid
     bool GlobalPlanner::isStartAndGoalCellsValid(int startCell, int goalCell){
          bool isvalid=true;
          //bool isFreeStartCell = isFreeCell(startCell);
          bool isFreeGoalCell = isFreeCell(goalCell);
          if (startCell == goalCell){
             isvalid = false;
          }
          else{
          
             if (!isFreeGoalCell){ //goal is an obstacle
                isvalid = false;
             }
             else{
                if(findNeighborCell(startCell).size() == 0){
                   isvalid = false;
                }
                if (findNeighborCell(goalCell).size() == 0){
                   isvalid = false;
                }
             }
             
          }
          return isvalid;
     }
     
     
     /***   Function relative with cell cooridnate **/
     /***********************************************/
     void GlobalPlanner::getCoordinate(float& x, float& y){
          x = x - originX;
          y = y - originY;
     }
     bool GlobalPlanner::iscellInsideMap(float x, float y){
          bool valid = true;
          if ((x > width*reso)||(y > height*reso))
              valid = false;
          return valid;
     }
     
     //Get cell index in OGM[]
     int GlobalPlanner::getCellIndex(int i, int j){
         return j*width + i;
     }
     int GlobalPlanner::converttoCellIndex(float x, float y){
         //int cellindex;
         //float X_ = x/reso;
         //float Y_ = y/reso;
         //cellindex = getCellIndex(static_cast<int>(X_), static_cast<int>(Y_));
         //return cellindex;
         return getCellIndex(static_cast<unsigned int>(x/reso), static_cast<unsigned int>(y/reso));
     }
     
     int GlobalPlanner::getRowIndex(int index){
         return index/width;
     }
     int GlobalPlanner::getColIndex(int index){
         return index%width;
     }
     
     
     //***********Function relative with map coordinate ******* //
     //*********************************************************//
     
     void GlobalPlanner::converttoMap(int index, float& x, float& y){
          x = (getColIndex(index)+0.5)*reso + originX;
          y = (getRowIndex(index)+0.5)*reso + originY;   
     
     }
     

     
     
     
     /****** Algorithm A* ******/
     /**************************/
     vector<int> GlobalPlanner::planner(int startCell, int goalCell){
        float* gcost;//Initilaize pointer gcost
        gcost = new float [mapsize];//Store move cost of cell
        
        for (unsigned int i = 0; i<mapsize;i++){
             gcost[ i ] = infinity;//initialize gcost of cell unvisited is infinity, if visited it a specific value
        } 
        
        vector<int> path_;
        timespec time1, time2;
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
        path_ = findpath(startCell, goalCell, gcost);
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
        
        cout << "Time to generate path: "<<(diff(time1,time2).tv_sec)*1e3 + (diff(time1,time2).tv_nsec)*1e-6 << "(microseconds)" <<endl;
        
        delete[] gcost;//delete gcost
        
        return path_;
     }
     
     
     
     //Function findpath of Relaxed A* Algorithm//
     vector<int> GlobalPlanner::findpath(int startCell, int goalCell, float gcost[]){
        vector<int> path_;
        vector<int> neighborCell;
        multiset <cell_, cmp> OPEN;
        cell_ CE;
        
       //BEGIN
       gcost[startCell] = 0;
       CE.index = startCell;
       CE.fcost = gcost[CE.index] + calculateHcost(CE.index, goalCell);
       
       OPEN.insert(CE);
       int currentCell;
       
       //Process inflates gcost from startCell
       while (!OPEN.empty()&& gcost[goalCell]==infinity){
           currentCell = OPEN.begin()->index;
           OPEN.erase(OPEN.begin());
           
           if (currentCell == goalCell){
               break;
           }
           else{
               neighborCell = findNeighborCell(currentCell);//Finding neighbor free cell
               for (unsigned int i = 0; i < neighborCell.size(); i++){
                    if (gcost[neighborCell[i]] == infinity){//neighbor is unvisited
                        gcost[neighborCell[i]] = gcost[currentCell] + getMoveCost(currentCell, neighborCell[i]);
                        addneighbortoOpen(OPEN, neighborCell[i], goalCell, gcost);
                    }//end if
               }//end for
               
           }// end if-else
           //neighborCell.clear();
       }//end while
       
       // Reach goal, the next step: constructing path after process inflating
       if (gcost[goalCell] != infinity)// reach goal
       {
           //cout << "Found Path"<<endl;
           ROS_INFO("Found Path");
           path_ = constructPath(startCell, goalCell, gcost);
       }
       return path_;

        
     }
     
     
     //Constructing path after inflating gcost from startCell to goalCell
     vector<int> GlobalPlanner::constructPath(int startCell, int goalCell, float gcost[]){
        vector<int> repath_;
        vector<int> path_;
        
        vector<int> neighborCell;
        vector<float> gcostNeighbor;
        
        int currentCell;
        int pose;
        currentCell = goalCell;
        repath_.push_back(goalCell);
        //repath_.insert(repath_.begin() + path_.size(), goalCell);
        
        while(currentCell != startCell){
             neighborCell = findNeighborCell(currentCell);
             
             for (unsigned int i = 0; i < neighborCell.size(); i++){
                 gcostNeighbor.push_back( gcost[neighborCell[i]] );
             }//end for
             
             pose = distance(gcostNeighbor.begin(), min_element(gcostNeighbor.begin(), gcostNeighbor.end())); // return mumber of elements between begin and min element// min gcost
             currentCell = neighborCell[pose];
             repath_.push_back(currentCell);
             //repath_.insert(repath_.begin() + repath_.size(), currentCell);
             //neighborCell.clear();
             
             gcostNeighbor.clear();
                
        }//end while
        
        //repath_.push_back(startCell);
        //Rearrange repath_
        for (unsigned int i=0; i<repath_.size(); i++){
             //path_.insert(path_.begin() + path_.size(), repath_[repath_.size() - i -1]);
             path_.push_back(repath_[repath_.size() - i -1]);
        }
        
        if (path_.size() > 2){
            path_ = reconstructPath(path_);
        }
        //Reconstruct one more times to find a optimal path
        if (path_.size() > 2){
            path_ = reconstructPath(path_);
        }
        return path_;
     }
     
     //Reconstruct path created by RA star
     
     vector<int> GlobalPlanner::reconstructPath(vector<int> path_){
         vector<int> repath_;
         int cell = path_[0];
         
         repath_.push_back(cell);

         for (unsigned int i=2; i<path_.size(); i++){
             if(!freeZone(cell, path_[i])){
                //cout << 1<<endl;
                cell = path_[i-1];//let path far away from obstacle
                repath_.push_back(cell);
             }
         }//end for
         repath_.push_back(path_[path_.size()-1]);
         return repath_;
     }
     
     //Check free zone between cell1 and cell2
     
     bool GlobalPlanner::freeZone(int cell1, int cell2){
         int row1 = getRowIndex(cell1);//y
         int col1 = getColIndex(cell1);//x
         
         int row2 = getRowIndex(cell2);
         int col2 = getColIndex(cell2);
         float k = 1;
         if (col1 != col2){
            k = static_cast<float>(row2 - row1)/(col2 - col1);
         }
         int cell;
         
         if (col1 < col2){
            for (int i = 1; i < (col2 - col1); i++){
                cell = getCellIndex(col1 + i, static_cast<int>( round(k*i + row1) ) );
                if (!isFreeCell(cell)){
                    return false;
                }//end if
            }//end for
         }
         else if (col1 > col2){
            for (int i = 1; i < (col1 - col2); i++){
                cell = getCellIndex(col1 - i,static_cast<int>(round(-k*i + row1)));
                if (!isFreeCell(cell)){
                    return false;
                }//end if
            }//end for
         }//end if
         else{ // col1 = col2
         
            if (row1 < row2){
                for (int i = 1; i < (row2 - row1); i++){
                    cell = getCellIndex(col1, row1 + i);
                    if (!isFreeCell(cell)){
                       return false;
                    }//end if
                }//end for
            }
            else{
                for (int i = 1; i < (row1 - row2); i++){
                    cell = getCellIndex(col1, row1 - i);
                    if (!isFreeCell(cell)){
                        return false;
                    }//end if
                }//end for
            }//end if else
            
         }//end if elseif else
         return true;
         
     }
     
     //Calculate Heuristic cost
     float GlobalPlanner::calculateHcost(int Cell, int goalCell){
         int row = getRowIndex(Cell);
         int col = getColIndex(Cell);
         
         int rowg = getRowIndex(goalCell);
         int colg = getColIndex(goalCell);
         
         return (abs(row-rowg) + abs(col - colg));
         //return sqrt( pow((rowg - row), 2) + pow((colg - col), 2) );
     }
     
     //Find Neighbor Free Cell
     inline vector<int> GlobalPlanner::findNeighborCell(int currentIndex){
         int row = getRowIndex(currentIndex);
         int col = getColIndex(currentIndex);
         int neighborIndex;
         vector<int> neighborCell;
         
         for (int i=-1; i<2; i++){
              for(int j=-1; j<2; j++){
                   if ((row+i >= 0) && (row+i<height) && (col+j >= 0) && (col+j < width) && !(i==0 && j==0)){
                       neighborIndex = getCellIndex(col+j, row+i); //*****
                       if (isFreeCell(neighborIndex)){
                           neighborCell.push_back(neighborIndex);
                       }//end if
                   }//end if
              }//end for
         }//end for
         return neighborCell;
     }
     
     bool GlobalPlanner::isFreeCell(int index){
          return OGM[index];
     }
     
     //Add neighborCell to OPEN list
     void GlobalPlanner::addneighbortoOpen(multiset<cell_, cmp>& OPEN, int neighborCell, int goalCell, float gcost[] ){
          cell_ CE;
          CE.index = neighborCell;
          CE.fcost = gcost[neighborCell] + calculateHcost(neighborCell, goalCell);
          OPEN.insert(CE);
     }
     
     //Get move cost of from current cell
     float GlobalPlanner::getMoveCost(int startCell, int neighborCell){
          int row = getRowIndex(startCell);
          int col = getColIndex(startCell);
          
          int rown = getRowIndex(neighborCell);
          int coln = getColIndex(neighborCell);
          
          float movecost = 1.0;
          if ((rown == row +1 && coln == col +1) || (rown == row +1 && coln == col -1) || (rown == row -1 && coln == col +1) || (rown == row -1 && coln == col -1)){
              movecost = 1.4;
          }//end if
          /*else{
              if ((rown == row && coln == col +1) || (rown == row && coln == col -1) || (rown == row +1 && coln == col) || (rown == row -1 && coln == col)){
                  movecost = 1.0;
              }
          }*/
          return movecost;
     }
     
};
