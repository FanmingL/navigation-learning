//
// Created by erdou on 18-10-13.
//

#ifndef RSCAR_UTIL_H
#define RSCAR_UTIL_H
namespace rs{
    namespace common{
        class WatchDog{
        public:
            WatchDog(int _max_count):max_count(_max_count),count(0){

            }
            bool CheckDog(){
                return ((++count) < max_count );
            }
            void FeedDog(){
                count = 0;
            }
        private:
            int max_count;
            int count;
        };
    }
}
#endif //RSCAR_UTIL_H
