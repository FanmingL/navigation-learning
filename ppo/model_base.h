//
// Created by Fanming Luo on 2019/1/26.
//

#ifndef TD3_MODEL_BASE_H
#define TD3_MODEL_BASE_H

#include <torch/nn/modules.h>
#include <string>
#include <vector>
#include <memory>

namespace rs {
    namespace al {
        class ModelBase : public torch::nn::Module {
        public:
            ModelBase(const torch::Device &d, const std::string &_name) : it_name(_name), device(d) {

            }

            virtual ~ModelBase() = default;

            void CopyFromHard(std::shared_ptr<ModelBase> &model) {
                for (int i = 0; i < fc.size(); ++i) {
                    (this->fc[i])->weight = model->fc[i]->weight.detach();
                }
            }

            void CopyFromHard(ModelBase &model){
                for (int i = 0; i < fc.size(); ++i) {
                    (this->fc[i])->weight = model.fc[i]->weight.detach();
                }
            }

            void CopyFromSoft(std::shared_ptr<ModelBase> &model, float rho) {
                for (int i = 0; i < fc.size(); ++i) {
                    (this->fc[i])->weight =
                            (this->fc[i])->weight.detach() * rho + (1 - rho) * model->fc[i]->weight.detach();
                }
            }

            void CopyFromSoft(ModelBase &model, float rho) {
                for (int i = 0; i < fc.size(); ++i) {
                    (this->fc[i])->weight =
                            (this->fc[i])->weight.detach() * rho + (1 - rho) * model.fc[i]->weight.detach();
                }
            }


            void SaveTo(const std::string &dir, const int &index) {
                std::string path = dir + it_name + std::to_string(index) + ".pt";
                std::ofstream of(path, std::ios::trunc | std::ios::binary);
                torch::serialize::OutputArchive os;
                save(os);
                os.save_to(of);
                std::cout << "Save at " << path << std::endl;
            }

            void LoadFrom(const std::string &dir, const int &index) {
                std::string path = dir + it_name + std::to_string(index) + ".pt";
                torch::serialize::InputArchive is;
                std::ifstream _if(path, std::ios::binary);
                is.load_from(_if);
                load(is);
                std::cout << "Load from " << path << std::endl;
            }

            std::vector<torch::nn::Linear> fc;
            torch::Device device;
            std::string it_name;
        };
    }
}
#endif //TD3_MODEL_BASE_H
