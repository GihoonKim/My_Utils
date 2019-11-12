#include <limits>
#include <fstream>
#include <stdexcept>
#include <string>
#include <iostream>
#include <sstream>
#include <array>
#include <map>
#include <vector>

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



int main(){

  DBC db("Lane_Info.dbc");
  
  return 0;
    
}
