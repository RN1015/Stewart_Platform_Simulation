#include <iostream>
#include <stack>
#include <string.h>

using namespace std;

int solution(char *string) {
    stack<int> scores;
    int totalScore = 0;

    for (int i = 0; string[i] != '\0'; ++i) {
        if (string[i] == '(') {
            scores.push(0);
        } else {
            int currentScore = 0;
            while (!scores.empty() && scores.top() != 0) {
                currentScore += scores.top();
                scores.pop();
            }
            scores.pop();
            if (currentScore == 0) {
                currentScore = 1;
            } else {
                currentScore *= 2;
            }
            scores.push(currentScore);
        }
    }

    while (!scores.empty()) {
        totalScore += scores.top();
        scores.pop();
    }
    
    return totalScore;
}
