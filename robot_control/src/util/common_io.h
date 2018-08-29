//
// Created by erdou on 18-8-29.
//

#ifndef ROBOT_CONTROL_IO_H
#define ROBOT_CONTROL_IO_H
#include <google/protobuf/message.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <fcntl.h>
#include <unistd.h>
#include "stdio.h"
namespace robot_io{
    template <class T>
    bool read_proto_from_text(const char *path, T* proto){
    using google::protobuf::io::FileInputStream;
    using google::protobuf::io::FileOutputStream;
    using google::protobuf::io::ZeroCopyInputStream;
    using google::protobuf::io::CodedInputStream;
    using google::protobuf::io::ZeroCopyOutputStream;
    using google::protobuf::io::CodedOutputStream;
    using google::protobuf::Message;

    int fd = open(path, O_RDONLY);
    if (fd == -1){
        printf("No Such File At %s", path);
        return false;
    }
    auto input = new FileInputStream(fd);
    bool success = google::protobuf::TextFormat::Parse(input, proto);
    delete input;
    close(fd);
    return success;
    }
}
#endif //ROBOT_CONTROL_IO_H
