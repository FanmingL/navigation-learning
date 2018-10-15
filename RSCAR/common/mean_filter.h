//
// Created by erdou on 18-10-13.
//

#ifndef RSCAR_MEAN_FILTER_H
#define RSCAR_MEAN_FILTER_H

#include <vector>

namespace rs{
    namespace common{
        template<typename T>
        class mean_filter{
        public:
            explicit mean_filter(const T &data_init, int _length = 6) : num(1), index(1), length(_length){
                buffer.push_back(data_init);
                sum = data_init;
                last_out = data_init;
            }
            T Run(const T &in){
                if (num == length)
                {
                    if (index == length)index = 0;
                    sum = sum - buffer[index];
                    buffer[index] = in;
                    sum = sum + buffer[index];
                    index++;
                }
                else
                {
                    sum = sum + in;
                    buffer.push_back(in);
                    index++;
                    num++;
                }
                return sum / num;
            }

            T RunAndCalVel(const T &in, T &velocity){
                T now = Run(in);
                velocity = now - last_out;
                last_out = now;
                return now;
            }

        private:
            std::vector<T> buffer;
            int index;
            int num;
            int length;
            T sum;
            T last_out;
        };
    }
}

#endif //RSCAR_MEAN_FILTER_H
