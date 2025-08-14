#ifndef __HX_PRINTF_UTILS_H
#define __HX_PRINTF_UTILS_H

#include <iostream>
#include <string>
#include <Eigen/Eigen>

using namespace std;

namespace printf_utils {

// Print colored text for better console output
enum Color {
    BLACK = 30,
    RED = 31,
    GREEN = 32,
    YELLOW = 33,
    BLUE = 34,
    MAGENTA = 35,
    CYAN = 36,
    WHITE = 37
};

// Print with color
void print_color(const string& text, Color color = WHITE) {
    cout << "\033[1;" << color << "m" << text << "\033[0m";
}

// Print with color and newline
void println_color(const string& text, Color color = WHITE) {
    print_color(text + "\n", color);
}

// Print Eigen vectors with formatting
void print_eigen_vector(const string& name, const Eigen::Vector3d& vec, int precision = 3) {
    cout << name << ": [";
    cout << fixed << setprecision(precision);
    cout << vec(0) << ", " << vec(1) << ", " << vec(2);
    cout << "]" << endl;
}

// Print Eigen matrices with formatting
void print_eigen_matrix(const string& name, const Eigen::Matrix3d& mat, int precision = 3) {
    cout << name << ":" << endl;
    cout << fixed << setprecision(precision);
    for(int i = 0; i < 3; i++) {
        cout << "[";
        for(int j = 0; j < 3; j++) {
            cout << mat(i,j);
            if(j < 2) cout << ", ";
        }
        cout << "]" << endl;
    }
}

// Print quaternion with formatting
void print_quaternion(const string& name, const Eigen::Quaterniond& q, int precision = 3) {
    cout << name << ": [w:" << fixed << setprecision(precision) << q.w() 
         << ", x:" << q.x() << ", y:" << q.y() << ", z:" << q.z() << "]" << endl;
}

// Print separator line
void print_separator(const string& title = "") {
    cout << "========================================" << endl;
    if (!title.empty()) {
        cout << "  " << title << endl;
        cout << "========================================" << endl;
    }
}

// Print warning message
void print_warning(const string& message) {
    println_color("[WARNING] " + message, YELLOW);
}

// Print error message
void print_error(const string& message) {
    println_color("[ERROR] " + message, RED);
}

// Print info message
void print_info(const string& message) {
    println_color("[INFO] " + message, GREEN);
}

// Print debug message
void print_debug(const string& message) {
    println_color("[DEBUG] " + message, CYAN);
}

} // namespace printf_utils

#endif