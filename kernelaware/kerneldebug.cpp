#include "mark3.h"
#include "kernelaware.h"

using namespace Mark3;

extern "C" {
void DebugPrint(const char* szString_)
{
    KernelAware::Print(szString_);
}
} // namespace Mark3

