#include <thread>
#include <iostream>

float shared_variable = 0;

void increment_function() {
    for (int i = 0; i < 1000000; ++i) {
          shared_variable += 1;
    }
}

// int main() {
//     std::thread t1(increment_function);
//     std::thread t2(increment_function);

//     t1.join();
//     t2.join();

//     std::cout << "Final value: " << shared_variable << std::endl;
//     return 0;
// }
