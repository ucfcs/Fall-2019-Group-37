#include "ros/ros.h"

/* Purpose is to read in the nav_msgs/OccupancyGrid and
 * nav_msgs/MapMetaData, and make decisions on which cell to travel to
 */

int main(int argc, char **argv)
{

    //Keep copy of map for over time analysis (Have we travelled to the cell)

    //Keep copy of vector of GPS locations

    /*Initialize*/ 

    ros::init(argc, argv, "Decision");

    ros::NodeHandle nh;

    //TODO Synchronize these two subscribers

    ros::Subscriber subMap = nh.subscribe("map", 1000, mapCallback);
    ros::Subscriber subMapMetaData = nh.subscribe("map_metadata", 1000, mapCallback);
    ros::Subscriber subStop = nh.subscribe("stop", 1000, stopCallback);
    ros::Subscriber subGps = nh.subscribe("fix", 1000, gpsCallback);
    ros::Publisher movementPub = nh.advertise<egoat::SetMotorSpeed>("motors", 100);

    /* Stop Subscriber */
    
    //Did the rover stop? Do we really need this info....
        //Move backwards from direction there is an obstacle very close 
        //Manuver around object we stopped at

    /* Map and MapMetaData Subscribers */

    void mapMetaDataCallback(const )
    {
        //Get info about the map (map size, origin, etc)
    }

    void mapCallback(const )
    {
        

        //Create map to make a decision 

        //Cells in all 4 directions (Exclude cell behind if possible)

        //Calculate values for each cell based on:
            // Probability of Object in Cell
            // Is the cell aligned/continues cutting algorithm ?
            // Has the cell been visitied before?
            // Is the cell within the boundaries

            //Take two best cells, and check 3 directions and figure out which to use

        //Add move to a two step movement queue
    }

    void gpsCallback()
    {
        //Store GPS location

        //Figure out averaged current position

        //Eventually calculate distance travelled, approximate velocity, etc
    }


    /* Movement Publisher */

    //Move straight ahead
    

    //Move left then straight ahead

    //Move right then straight ahead

    //Move backwards

    //Turn around
}