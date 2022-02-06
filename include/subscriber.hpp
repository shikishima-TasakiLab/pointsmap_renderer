#pragma once
#include <ros/ros.h>
#include <mutex>

template<class msg_T>
class Subscriber
{
public:
    Subscriber(
        ros::NodeHandle &node_handle,
        const std::string &topic,
        const uint32_t queue_size
    ) {
        this->_mtx = new std::mutex();
        this->_sub = new ros::Subscriber(node_handle.subscribe<msg_T>(topic, queue_size, &Subscriber::_callback, this));
    }

    ~Subscriber() {
        delete this->_sub;
        delete this->_mtx;
    }

    boost::shared_ptr<const msg_T> get_msg() {
        std::lock_guard<std::mutex> lock(*this->_mtx);
        return this->_msg;
    }

private:
    ros::Subscriber *_sub;
    boost::shared_ptr<const msg_T> _msg;
    std::mutex *_mtx;

    void _callback(const boost::shared_ptr<const msg_T> &msg) {
        std::lock_guard<std::mutex> lock(*this->_mtx);
        this->_msg = msg;
    }
};
