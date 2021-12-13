cppcheck --language=c++ --std=c++11 -I include/ --suppress=missingIncludeSystem  $( find . -name \*.h -or -name \*.cpp | grep -vE -e "^./build/" -e "^./vendor/") > results/cppcheck_test.txt
