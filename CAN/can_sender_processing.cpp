
# include <stdio.h>
# include <ros/ros.h>
#include <limits>
#include <fstream>
#include <stdexcept>
#include <string>
# include <iostream>
# include <sys/ioctl.h>
# include <fcntl.h>
# include <net/if.h>
# include <linux/can.h>
# include <linux/can/raw.h>
#include <sstream>
#include <array>
#include <map>
# include <vector>

#define MOTOROLA 0

using namespace std;

using namespace std;

static const std::array<std::string, 10> PREAMBLES =
{
  "VERSION",      // VERSION
  "BS_:",         // BUS CONFIG
  "BU_:",         // BUS NODES
  "BO_",          // MESSAGE
  "SG_",          // SIGNAL
  "CM_",          // COMMENT
  "VAL_",         // SIGNAL VALUE LIST
  "BA_DEF_",      // ATTRIBUTE DEFINITION
  "BA_DEF_DEF_",  // ATTRIBUTE DEFAULT VALUE
  "BA_"           // ATTRIBUTE VALUE
};


class DBC{

  public:
    DBC(const std::string &dbc_path);
    void parse(std::istream &reader);

    map<string, pair<int,int>> msg_list;
    map<string, pair<int,int>> sig_;

    map<string, map<string,pair<int,int>>> sig_list;

    int num1;
    int num2;
    bool is_msg = false;

    std::string msg_id_;
    std::string msg_name_;
    std::string msg_dlc_;
    
    std::string sig_name_;
    std::string temp_string;

    std::string version_;
    std::string bus_config_;
};

void DBC::parse(std::istream & reader){
  
  std::string line;

  while (std::getline(reader,line)){

    if (!line.empty() && line.rfind("\t") != 0) {

      std::istringstream iss(line);

      std::string preamble;
      iss>>preamble;
      if(preamble==PREAMBLES[0]){
        iss>>version_;
        std::cout<<"===================="<<std::endl;
        std::cout<<version_<<std::endl;

      }
      else if(preamble==PREAMBLES[3])
      {
        std::istringstream input(line);
      
        input.ignore(4);

        input>>msg_id_;
        std::cout<<"===================="<<std::endl;
        // std::cout<<msg_id_<<std::endl;
        input>>msg_name_;
        // std::cout<<msg_name_<<std::endl;
        input>>msg_dlc_;
        // std::cout<<msg_dlc_<<std::endl;
        // input>>else_;
        // std::cout<<else_<<std::endl;
        num1 = atoi(msg_id_.c_str());
        num2 = atoi(msg_dlc_.c_str());
        msg_list.insert(make_pair(msg_name_, make_pair(num1,num2)));
        is_msg = false;

         std::cout<<"msg_name is : " <<msg_name_<<std::endl;
        std::cout<<"msg_id is : " <<msg_list[msg_name_].first<<std::endl;
        std::cout<<"msg_dlc is : " <<msg_list[msg_name_].second<<std::endl;
      }
      else if(preamble==PREAMBLES[4])
      {
        std::istringstream input(line);

        input.ignore(5);
        std::cout<<" -------- "<<std::endl;
        input>>sig_name_;
        // std::cout<<sig_name_<<std::endl;
        input>>temp_string;
        input >> temp_string;

        auto bar = temp_string.find("|");
        auto at = temp_string.find("@");

        unsigned char start_bit_;
        unsigned char length_;
        if (bar != std::string::npos && at != std::string::npos) {
          start_bit_ = static_cast<unsigned char>(std::stoul(temp_string.substr(0, bar)));
          length_ = static_cast<unsigned char>(std::stoul(temp_string.substr(bar + 1, at - bar - 1)));
        }


        // std::cout<<(int)start_bit_<<std::endl;
        // std::cout<<(int)length_<<std::endl;

        
        if(!is_msg)
        {
          sig_.insert(make_pair(sig_name_, make_pair((int)start_bit_,(int)length_)));
          sig_list.insert(make_pair(msg_name_,sig_));
          sig_.clear();
          is_msg = true;
        }
        else{
          sig_list[msg_name_].insert(make_pair(sig_name_, make_pair((int)start_bit_,(int)length_)));
          
        }

        std::cout<<"sig name is  "<<sig_name_<<std::endl;
        std::cout<<"start bit is  "<<sig_list[msg_name_][sig_name_].first<<std::endl;
        std::cout<<"length is  "<<sig_list[msg_name_][sig_name_].second<<std::endl;

      }
      

      
    }
  }
}

DBC::DBC(const std::string & dbc_path){
  std::ifstream file_reader;
  
  
  file_reader.open("Lane_info.dbc");
  if (file_reader.is_open()) {

    DBC::parse(file_reader);
    // std::cout<<DBC::sig_list["LANE_INFO_A:"].size()<<std::endl;
    // std::cout<<DBC::sig_list["CAR20:"].size()<<std::endl;


  }
}

static void clear_msg (unsigned char *msg)
{
  memset (msg, 0, 8);
} /* clear_msg */

static int store_float (unsigned char *msg, int pos, int len, float f_value, int motorola)
{
  int bpos = 0, bit = 0;
  // int len = sizeof (float) * 8;
  unsigned int value = 0;
  if (sizeof (value) != sizeof (f_value)) return -1;
  memcpy (&value, &f_value, sizeof (value));
  if ((pos >= 64) || (pos < 0) || (len < 1) || ((pos + len - 1) >= 64)) return -1;
  if (!msg) return -1;
  bpos = pos / 8;
  bit = pos % 8;
  while (len > 0) {
    unsigned int mask = 0, v;
    if (len == 1) mask = 0x01;
    else if (len == 2) mask = 0x03;
    else if (len == 3) mask = 0x07;
    else if (len == 4) mask = 0x0f;
    else if (len == 5) mask = 0x1f;
    else if (len == 6) mask = 0x3f;
    else if (len == 7) mask = 0x7f;
    else if (len >= 8) mask = 0xff;
    mask = (mask << bit) & 0xff;
    if (motorola) {
      int sh = len - (8 - bit);
      if (sh < 0) sh = 0;
      v = ((value >> sh) << bit) & 0xff;
    }
    else v = (value << bit) & 0xff;
    msg [bpos] = (msg [bpos] & ~mask) | (v & mask);

    if (!motorola) value = value >> (8 - bit);
    len -= 8 - bit;
    bit = 0;
    ++bpos;
  }
  return 0;
} /* store_float */


class can_sender{

  public:
    can_sender();

    void callback_lane_info(const mmc_msgs::to_control_team_from_local_msg &msg);
    int open_port(const char* port);

    int read_can_port = 4;
    int soc;
    int alive_count = 0;

    std::vector<string> db_lane;
    std::vector<string> db_pos;

    struct can_frame frame;

};

can_sender::can_sender()
{
    can_sender::open_port("can4");

    db_lane.push_back("LANE_INFO_A:");
    db_lane.push_back("LANE_INFO_B:");

    db_pos.push_back("CAR_EGO_A:");
    db_pos.push_back("CAR_EGO_B:");
    db_pos.push_back("CAR_EG)_SD:");
}

int can_sender::open_port(const char *port)
{
    struct ifreq ifr;
    struct sockaddr_can addr;

    /* open socket */
    soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if(soc<0)
    {
        return (-1);
    }

    addr.can_family = AF_CAN;
    strcpy(ifr.ifr_name,port);

    if(ioctl(soc, SIOCGIFINDEX, &ifr)<0)
    {
        return(-1);
    }

    addr.can_ifindex = ifr.ifr_ifindex;

    fcntl(soc, F_SETFL, O_NONBLOCK);

    if(bind(soc,(struct sockaddr*)&addr, sizeof(addr))<0)
    {
        return(-1);
    }

    cout<<"CAN is Connected"<<endl;

    return 0;

}

void can_sender::Send_msg(const mmc_msgs::to_control_team_from_local_msg &callback_msg)
{

  unsigned char msg[8];
  int start_bit;
  int length_;
  string sig_name;
  string msg_name;

  can_sender::alive_count +=1;
  if(can_sender::alive_count>255) {can_sender::alive_count = 0;}

  /* LANE_INFO_A */

  msg_name = can_sender::db_lane.at(0);
  frame.can_id = DBC::msg_list[msg_name].first;
  frame.can_dlc = DBC::msg_list[msg_name].second;
  clear_msg(msg);

  sig_name = "LANE_ID";
  start_bit = DBC::sig_list[msg_name][sig_name].first;
  length_ = DBC::sig_list[msg_name][sig_name].second;
  store_float(msg, start_bit, length_, callback_msg.lane_id, MOTOROLA);

  sig_name = "ALLIVE_COUNT";
  start_bit = DBC::sig_list[msg_name][sig_name].first;
  length_ = DBC::sig_list[msg_name][sig_name].second;
  store_float(msg, start_bit, length_, can_sender::alive_count, MOTOROLA);

  // frame.data[0] = 0x11;  
  frame.data = msg;

  nbytes = write(soc, &frame, sizeof(struct can_frame));

  /* LANE_INFO_B */

  msg_name = can_sender::db_lane.at(1);
  frame.can_id = DBC::msg_list[msg_name].first;
  frame.can_dlc = DBC::msg_list[msg_name].second;
  clear_msg(msg);

  sig_name = "M_ENTER";
  start_bit = DBC::sig_list[msg_name][sig_name].first;
  length_ = DBC::sig_list[msg_name][sig_name].second;
  store_float(msg, start_bit, length_, callback_msg.distance_to_entry_end, MOTOROLA);

  sig_name = "M_EXIT";
  start_bit = DBC::sig_list[msg_name][sig_name].first;
  length_ = DBC::sig_list[msg_name][sig_name].second;
  store_float(msg, start_bit, length_, callback_msg.distance_to_exit_start, MOTOROLA);

  frame.data = msg;
  nbytes = write(soc, &frame, sizeof(struct can_frame));

  /* CAR_EGO_A */

  msg_name = can_sender::db_pos.at(0);
  frame.can_id = DBC::msg_list[msg_name].first;
  frame.can_dlc = DBC::msg_list[msg_name].second;
  clear_msg(msg);

  sig_name = "X";
  start_bit = DBC::sig_list[msg_name][sig_name].first;
  length_ = DBC::sig_list[msg_name][sig_name].second;
  store_float(msg, start_bit, length_, callback_msg.host_east, MOTOROLA);

  sig_name = "Y";
  start_bit = DBC::sig_list[msg_name][sig_name].first;
  length_ = DBC::sig_list[msg_name][sig_name].second;
  store_float(msg, start_bit, length_, callback_msg.host_north, MOTOROLA);

  frame.data = msg;
  nbytes = write(soc, &frame, sizeof(struct can_frame));

   /* CAR_EGO_B */

  msg_name = can_sender::db_pos.at(1);
  frame.can_id = DBC::msg_list[msg_name].first;
  frame.can_dlc = DBC::msg_list[msg_name].second;
  clear_msg(msg);

  sig_name = "YAW";
  start_bit = DBC::sig_list[msg_name][sig_name].first;
  length_ = DBC::sig_list[msg_name][sig_name].second;
  store_float(msg, start_bit, length_, callback_msg.host_yaw, MOTOROLA);

  frame.data = msg;
  nbytes = write(soc, &frame, sizeof(struct can_frame));

  /* CAR_EGO_SD */

  msg_name = can_sender::db_pos.at(2);
  frame.can_id = DBC::msg_list[msg_name].first;
  frame.can_dlc = DBC::msg_list[msg_name].second;
  clear_msg(msg);

  sig_name = "EGO_S";
  start_bit = DBC::sig_list[msg_name][sig_name].first;
  length_ = DBC::sig_list[msg_name][sig_name].second;
  store_float(msg, start_bit, length_, callback_msg.station, MOTOROLA);

  sig_name = "EGO_D";
  start_bit = DBC::sig_list[msg_name][sig_name].first;
  length_ = DBC::sig_list[msg_name][sig_name].second;
  store_float(msg, start_bit, length_, callback_msg.lateral_offset, MOTOROLA);

  frame.data = msg;
  nbytes = write(soc, &frame, sizeof(struct can_frame));
  
}


void can_sender::callback_lane_info(const mmc_msgs::to_control_team_from_local_msg &msg)
{
  can_sender::Send_msg(msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "can_sender");
    ros::NodeHandle node("~");

    DBC db("Lane_Info.dbc");
    can_sender cp();

    ros::Subscriber sub = node.subscribe<mmc_msgs::to_control_team_from_local_msg>("/localization/to_control_team",1.&can_sender::callback_lane_info)

    ros::spin();

    return 0;
}
