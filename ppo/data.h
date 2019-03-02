/*================================================================
*   Copyright (C) 2019 * Ltd. All rights reserved.
*   
*   File name   : data.h
*   Author      : FanmingL
*   Created date: 2019-02-26 13:24:56
*   Description : 
*
*===============================================================*/


#ifndef _DATA_H
#define _DATA_H

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

namespace rs{
namespace cs{
class OneCarInformation{
public:
	int index;
	float x, y, vx, vy, next_vx, next_vy, target_x, target_y, angle;
	OneCarInformation(const int &index_, const float &x_, const float &y_, const float &vx_, const float &vy_, const float &angle_,
		const float &target_x_, const float &target_y_, const float &next_vx_, const float &next_vy_):
	index(index_), x(x_), y(y_), vx(vx_), vy(vy_), next_vx(next_vx_), next_vy(next_vy_), target_x(target_x_),
	target_y(target_y_), angle(angle_){}
	friend std::ostream & operator<<(std::ostream &out, const OneCarInformation &obj){
#if 1	
        out <<obj.index<<" "<<obj.x<<" "<<obj.y
        <<" "<<obj.vx<<" "<<obj.vy<<" "<<obj.angle
        <<" "<<obj.target_x<<" "<<obj.target_y
        <<" "<<obj.next_vx<<" "<<obj.next_vy;
#else
		out <<obj.index<<" x: "<<obj.x<<" y: "<<obj.y
        <<" vx: "<<obj.vx<<" vy: "<<obj.vy<<" angle: "<<obj.angle
        <<" target_x: "<<obj.target_x<<" target_y: "<<obj.target_y
        <<" next_vx: "<<obj.next_vx<<" next_vy: "<<obj.next_vy;
#endif
        return out;
    }	
};

class OneFrameCarInformation{
public:
	std::vector<OneCarInformation> data;
	int frame_index;	
	OneFrameCarInformation(const int& frame_index_):frame_index(frame_index_){}
	friend std::ostream & operator<<(std::ostream &out, const OneFrameCarInformation &obj){
		for (auto &item : obj.data){
			out << item << std::endl;
		} 
		return out;
	}
};

class AllFrameCarInformation
{
public:
	 std::vector<OneFrameCarInformation> data;
	 void init(std::ifstream &iff, const int &pixel_per_meter = 1, const int &index_base=0){
	 	std::string str;
		while(std::getline(iff, str)){
			auto &data_tmp = data.back();
			std::stringstream ss(str);
			std::vector<float> v;
			float t;
			while (ss >> t){
				v.push_back(t);
			}
			if (v.size() == 1){
				data.emplace_back(v[0]);
			}else{
				data_tmp.data.emplace_back(v[0] + index_base, v[1] * pixel_per_meter, v[2] * pixel_per_meter
						, v[3] * pixel_per_meter, v[4] * pixel_per_meter, v[5]
						, v[6] * pixel_per_meter, v[7] * pixel_per_meter, v[8] * pixel_per_meter
						, v[9] * pixel_per_meter);
			}
		}
		std::cout<<"Pre-read Data Init Done!\n";
	 }
	 friend std::ostream & operator<<(std::ostream &out, const AllFrameCarInformation &obj){
		for (auto &item : obj.data){
			out << item.frame_index << std::endl;
			out << item;
		} 
		return out;
	}

};
}
}
#endif //DATA_H
