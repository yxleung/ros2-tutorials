#include <iostream>
#include <memory>
#include <boost/type_index.hpp>
#include <string>


class A{
    public:
     int i = 0;
    
};
int main(int argc, char const *argv[])
{
    A* a1= new A();
    auto b = *a1;
    auto c = std::make_shared<A>();
    std::cout << c->i << std::endl;    
    
    std::string* s1 = new std::string(10,'9');
    std::cout << *s1 <<std::endl;

    auto d = std::make_shared<std::string>(10, '9');
    std::cout << *(d.get()) << std::endl;

    return 0;
}
