#include <iostream>
#include <random>
#include "TrafficLight.h"

template <typename T>
T MessageQueue<T>::receive()
{
       
    // perform vector modification under the lock
    std::unique_lock<std::mutex> uLock(_mutex);
    _cond.wait(uLock, [this] { return !_queue.empty(); }); // pass unique lock to condition variable

    // remove last vector element from queue
    T msg = std::move(_queue.back());
    _queue.pop_back();

    return msg; // will not be copied due to return value optimization (RVO) in C++
}

template <typename T>
void MessageQueue<T>::send(T &&msg)
{
    // perform vector modification under the lock
    std::lock_guard<std::mutex> uLock(_mutex);

    // add vector to queue
    _queue.push_back(std::move(msg));
    _cond.notify_one(); // notify client after pushing new Vehicle into vector
    
}

TrafficLight::TrafficLight()
{
    _currentPhase = TrafficLightPhase::red;
}

void TrafficLight::waitForGreen()
{
    while(true)
    {
        // popBack wakes up when a new element is available in the queue
        TrafficLightPhase phase = _mesageQueue.receive();
        if(phase == TrafficLightPhase::green)
            return;
    }
}

TrafficLightPhase TrafficLight::getCurrentPhase()
{
    return _currentPhase;
}

void TrafficLight::simulate()
{
    threads.emplace_back(std::thread(&TrafficLight::cycleThroughPhases, this));
}

// virtual function which is executed in a thread
void TrafficLight::cycleThroughPhases()
{
    // Generate a random number 4000 to 6000 ms
    std::random_device rd;
    std::mt19937 eng(rd());
    std::uniform_int_distribution<> distr(4000, 6000);

    double cycleDuration = distr(eng); // duration of a single simulation cycle in ms
    std::chrono::time_point<std::chrono::system_clock> lastUpdate;

    // init stop watch
    lastUpdate = std::chrono::system_clock::now();
    while(true)
    {
        // sleep at every iteration to reduce CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // compute time difference to stop watch
        long timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - lastUpdate).count();
        if (timeSinceLastUpdate >= cycleDuration)
        {
            // Toggle light phase:
            if(_currentPhase == TrafficLightPhase::green)
                _currentPhase = TrafficLightPhase::red;
            else
                _currentPhase = TrafficLightPhase::green;
            
            // Send current phase to the message queue
            _mesageQueue.send(std::move(_currentPhase));
           
            // reset stop watch for next cycle
            lastUpdate = std::chrono::system_clock::now();  
        }

    }
}