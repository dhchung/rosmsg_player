#ifndef DATAMANAGER_H
#define DATAMANAGER_H

#include <mutex>
#include <thread>
#include <condition_variable>
#include <queue>

struct DataManager{
    std::queue<std::pair<long double, std::string>> m_data_queue;
    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::thread m_thread;
    bool b_run;

    DataManager() {
        b_run = true;
    }
    ~DataManager() {
        b_run = false;
        m_cv.notify_all();
        
        if(m_thread.joinable()) {
            m_thread.join();
            std::cout<<"Joined in destructor"<<std::endl;
        }
    }

    void push(std::pair<long double, std::string> data) {
        m_mutex.lock();
        m_data_queue.push(data);
        m_mutex.unlock();
    }

    std::pair<long double, std::string> pop(){
        m_mutex.lock();
        std::pair<long double, std::string> data = m_data_queue.front();
        m_data_queue.pop();
        m_mutex.unlock();
        return data;
    }

    void clear()
    {
        std::queue<std::pair<long double, std::string>> empty;
        std::swap(m_data_queue, empty );
    }

};


#endif