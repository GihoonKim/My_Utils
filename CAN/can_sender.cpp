
# include <stdio.h>
# include <ros/ros.h>
#include <limits>
#include <fstream>
#include <ostream>
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
# include <cstring>
# include <typeinfo>
#include <time.h>
# include <mmc_msgs/to_control_team_from_local_msg.h>

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

    map<string, pair<int,unsigned char>> msg_list; //id, dlc

    map<string, pair<int,int>> sig_; //

    map<string, map<string,pair<int,int>>> sig_list; //ms

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
        // std::cout<<"===================="<<std::endl;
        // std::cout<<version_<<std::endl;

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
        // num2 = msg_dlc_;
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
  file_reader.open(dbc_path);

  if (file_reader.is_open()) {


    DBC::parse(file_reader);
    cout<<"Parser Finish"<<endl;
    std::cout<<"AT DBC   "<<DBC::sig_list["LANE_INFO_A:"].size()<<std::endl;
    // std::cout<<DBC::sig_list["CAR20:"].size()<<std::endl;


  }
}

static void clear_msg (unsigned char *msg)
{
  memset (msg, 0, 8);
} /* clear_msg */

static int store_float (unsigned char *msg, int pos, int len, float f_value, int motorola)
{
  //value = 15
  int bpos = 0, bit = 0;
  // int len = sizeof (float) * 8;
  unsigned int value = 0;

  if (sizeof (value) != sizeof (f_value)) {cout<<"size mismatch"<<endl; return -1;}
  
  memcpy (&value, &f_value, sizeof (value));
  
  if ((pos >= 64) || (pos < 0) || (len < 1) || ((pos + len - 1) >= 64)) return -1;
  if (!msg) return -1;

  bpos = pos / 8;    //3
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
    

    void callback_lane_info(const mmc_msgs::to_control_team_from_local_msg::ConstPtr &msg);
    int open_port(const char* port);
    int Encoding(unsigned char* msg, std::string msg_name, std::vector<pair<std::string,double>> value);

    int soc;
    int alive_count = 0;
    
    struct sockaddr_can addr;
    struct ifreq ifr;
    
    std::vector<string> db_lane;
    std::vector<string> db_pos;
    
    // DBC db("Lane_info.dbc");

    DBC db;

    can_sender(DBC db_);
    

};

can_sender::can_sender(DBC db_) : db(db_)
{
    can_sender::open_port("can0");

    db_lane.push_back("LANE_INFO_A:");
    db_lane.push_back("LANE_INFO_B:");

    db_pos.push_back("CAR_EGO_A:");
    db_pos.push_back("CAR_EGO_B:");
    db_pos.push_back("CAR_EGO_SD:");
}

int can_sender::Encoding(unsigned char* msg, std::string msg_name, std::vector<pair<std::string,double>> value){

  int num_sig = db.sig_list[msg_name].size();
  std::cout<<"Msg name is ["<<msg_name<<"]"<<" num of signal in msg : "<<num_sig<<std::endl; ///////////////////////////////////////////////////////////////////////
  for (int i=0; i<num_sig ;i++){
    string sig_name = value.at(i).first;
    std::cout<<i<<"th sig name is ["<<sig_name<<"]"<<endl;
    std::cout<<sig_name<<"'s startbit is : "<<db.sig_list[msg_name][sig_name].first<<"   "<<sig_name<<"'s length is : "<<db.sig_list[msg_name][sig_name].second<<endl;

  }
}

int can_sender::open_port(const char *port)
{
    /* open socket */
    soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    // soc = socket(PF_CAN, SOCK_DGRAM, CAN_BCM);

    if(soc<0)
    {
      cout<<"Open Socket error"<<endl;
      return (-1);
    }

    addr.can_family = AF_CAN;
    strcpy(ifr.ifr_name, port);

    if(ioctl(soc, SIOCGIFINDEX, &ifr)<0)
    {
      cout<<"Ioctl error"<<endl;
      return(-1);
    }

    addr.can_ifindex = ifr.ifr_ifindex;
    printf("%s at index %d\n", port, ifr.ifr_ifindex);

    

    if(bind(soc,(struct sockaddr*)&addr, sizeof(addr))<0)
    {
      cout<<"Bind error"<<endl;
      return(-1);
        
    }

    fcntl(soc, F_SETFL, O_NONBLOCK);

    cout<<"CAN is Connected"<<endl;

    return 0;

}


void can_sender::callback_lane_info(const mmc_msgs::to_control_team_from_local_msg::ConstPtr &callback_msg)
{
  
  int retval;
  unsigned char msg[8];
  int start_bit;
  int length_;
  string sig_name;
  string msg_name;
  struct can_frame frame;
  std::vector<pair<string,double>> msg_vec;

  clock_t start, end;

  start = clock();

  
  

  for (int check=0; check<15;check++){

  msg_vec.clear();
  msg_vec.push_back(make_pair("LANE_ID",1));
  msg_vec.push_back(make_pair("ALLIVE_CNT",11));
  can_sender::Encoding(msg, "LANE_INFO_A:", msg_vec);

  can_sender::alive_count +=1;
  if(can_sender::alive_count>255) {can_sender::alive_count = 0;}

  /* LANE_INFO_A */

  msg_name = can_sender::db_lane.at(0);
  frame.can_id = (unsigned int)db.msg_list[msg_name].first;
  frame.can_dlc = db.msg_list[msg_name].second;
  clear_msg(msg);


  sig_name = "LANE_ID";
  start_bit = db.sig_list[msg_name][sig_name].first;
  length_ = db.sig_list[msg_name][sig_name].second;
  store_float(msg, start_bit, length_, callback_msg->lane_id, MOTOROLA);


  sig_name = "ALLIVE_CNT";
  start_bit = db.sig_list[msg_name][sig_name].first;
  length_ = db.sig_list[msg_name][sig_name].second;
  store_float(msg, start_bit, length_, can_sender::alive_count, MOTOROLA);

  // frame.data[0] = 0x11;
  // frame.data[1] = 0x11;
  // frame.data[2] = 0x11;
  // frame.data[3] = 0x11;
  // frame.data[4] = 0x11;
  // frame.data[5] = 0x11;
  // frame.data[6] = 0x11;
  // frame.data[7] = 0x11;
  memcpy(frame.data, msg, sizeof(msg));

  retval = write(soc, &frame, sizeof(frame));
  // sendto
  // retval = sendto(soc, &frame, sizeof(struct can_frame), 0, (struct sockaddr*)&addr, sizeof(addr));  
  
  
  /* LANE_INFO_B */

  msg_name = can_sender::db_lane.at(1);
  frame.can_id = db.msg_list[msg_name].first;
  frame.can_dlc = db.msg_list[msg_name].second;
  clear_msg(msg);

  sig_name = "M_ENTER";
  start_bit = db.sig_list[msg_name][sig_name].first;
  length_ = db.sig_list[msg_name][sig_name].second;
  store_float(msg, start_bit, length_, callback_msg->distance_to_entry_end, MOTOROLA);

  sig_name = "M_EXIT";
  start_bit = db.sig_list[msg_name][sig_name].first;
  length_ = db.sig_list[msg_name][sig_name].second;
  store_float(msg, start_bit, length_, callback_msg->distance_to_exit_start, MOTOROLA);

  memcpy(frame.data, msg, sizeof(msg));
  retval = write(soc, &frame, sizeof(struct can_frame));
  // cout<<retval<<endl;
  /* CAR_EGO_A */

  msg_name = can_sender::db_pos.at(0);
  frame.can_id = db.msg_list[msg_name].first;
  frame.can_dlc = db.msg_list[msg_name].second;
  clear_msg(msg);

  sig_name = "X";
  start_bit = db.sig_list[msg_name][sig_name].first;
  length_ = db.sig_list[msg_name][sig_name].second;
  
  // std::cout<<callback_msg->host_east<<std::endl;
  // printf("%.2f",callback_msg->host_east);
  // std::cout<<typeid(callback_msg->host_east).name()<<std::endl;

  store_float(msg, start_bit, length_, callback_msg->host_east, MOTOROLA);
  
  sig_name = "Y";
  start_bit = db.sig_list[msg_name][sig_name].first;
  length_ = db.sig_list[msg_name][sig_name].second;

  // std::cout<<(callback_msg->host_north)<<std::endl;
  store_float(msg, start_bit, length_, callback_msg->host_north, MOTOROLA);

  memcpy(frame.data, msg, sizeof(msg));
  // retval = write(soc, &frame, sizeof(struct can_frame));
  retval = sendto(soc, &frame, sizeof(struct can_frame), 0, (struct sockaddr*)&addr, sizeof(addr));  
   /* CAR_EGO_B */

  msg_name = can_sender::db_pos.at(1);
  frame.can_id = db.msg_list[msg_name].first;
  frame.can_dlc = db.msg_list[msg_name].second;
  clear_msg(msg);

  sig_name = "YAW";
  start_bit = db.sig_list[msg_name][sig_name].first;
  length_ = db.sig_list[msg_name][sig_name].second;
  store_float(msg, start_bit, length_, callback_msg->host_yaw, MOTOROLA);

  memcpy(frame.data, msg, sizeof(msg));
  retval = write(soc, &frame, sizeof(struct can_frame));
  std::cout<<retval<<std::endl;

  /* CAR_EGO_SD */

  msg_name = can_sender::db_pos.at(2);
  frame.can_id = db.msg_list[msg_name].first;
  frame.can_dlc = db.msg_list[msg_name].second;
  clear_msg(msg);

  sig_name = "EGO_S";
  start_bit = db.sig_list[msg_name][sig_name].first;
  length_ = db.sig_list[msg_name][sig_name].second;
  store_float(msg, start_bit, length_, callback_msg->station, MOTOROLA);

  sig_name = "EGO_D";
  start_bit = db.sig_list[msg_name][sig_name].first;
  length_ = db.sig_list[msg_name][sig_name].second;
  store_float(msg, start_bit, length_, callback_msg->lateral_offset, MOTOROLA);

  memcpy(frame.data, msg, sizeof(msg));
  retval = write(soc, &frame, sizeof(struct can_frame));

  
  }
  end = clock();
  
  double result = (double)(end-start)/CLOCKS_PER_SEC;
  std::cout<<"total time is : "<<result*1000<<std::endl;

  // std::cout<<"Done : "<<end-start<<std::endl;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "can_sender");
    ros::NodeHandle node("~");

    DBC db("./src/control_response/can_sender/src/Lane_info.dbc");
    can_sender cp(db);

    // <mmc_msgs::to_control_team_from_local_msg>
    ros::Subscriber sub = node.subscribe<mmc_msgs::to_control_team_from_local_msg>("/localization/to_control_team",10, &can_sender::callback_lane_info, &cp);

    ros::spin();

    return 0;
}
