#include <iostream>
#include <boost/type_index.hpp>


int main(int argc, char const *argv[])
{
    int a = 1;
    auto b = a;
    auto& c = a;
    int* d = &a;
    auto* e = &a;
    std::cout << typeid(a).name() << std::endl;
    std::cout << typeid(b).name() << std::endl;
    std::cout << typeid(c).name() << std::endl;

    std::cout << "=====================" << std::endl;

    std::cout << boost::typeindex::type_id_with_cvr<decltype(a)>().pretty_name() << std::endl;
    std::cout << boost::typeindex::type_id_with_cvr<decltype(b)>().pretty_name() << std::endl;
    std::cout << boost::typeindex::type_id_with_cvr<decltype(c)>().pretty_name() << std::endl;
    std::cout << boost::typeindex::type_id_with_cvr<decltype(d)>().pretty_name() << std::endl;
    std::cout << boost::typeindex::type_id_with_cvr<decltype(e)>().pretty_name() << std::endl;
 
    std::cout << "=====================" << std::endl;
    std::cout << sizeof(a) << std::endl;
    std::cout << sizeof(b) << std::endl;
    std::cout << sizeof(c) << std::endl;
    std::cout << sizeof(d) << std::endl;
    std::cout << sizeof(e) << std::endl;


    return 0;
}
