#include <iostream>

class A{
    public:
    A(){
        std::cout<<"test"<<std::endl;
    }
    int i=10;
};

int main()
{
    A a;
    auto b =new A();
    A c = A();
    std::cout<<a.i<<std::endl;
    std::cout<<b->i<<std::endl;
    std::cout<<c.i<<std::endl;
    return 0;
}
