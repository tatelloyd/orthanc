#!/bin/bash
# Test IPC between YOLO and C++ tracker

echo "🔍 Testing Detection IPC"
echo "========================"
echo ""

# Check if detection file exists
if [ -f "detections.json" ]; then
    echo "✅ detections.json exists"
    echo ""
    echo "Contents:"
    cat detections.json | python3 -m json.tool 2>&1
    echo ""
    
    # Check if it's valid JSON
    if python3 -c "import json; json.load(open('detections.json'))" 2>/dev/null; then
        echo "✅ Valid JSON format"
        
        # Count detections
        COUNT=$(python3 -c "import json; print(len(json.load(open('detections.json'))))")
        echo "📊 Number of detections: $COUNT"
        
        # Show person detections
        PEOPLE=$(python3 -c "import json; data=json.load(open('detections.json')); print(sum(1 for d in data if d.get('label')=='person'))")
        echo "👤 People detected: $PEOPLE"
        
        if [ "$PEOPLE" -gt 0 ]; then
            echo ""
            echo "Person positions:"
            python3 -c "import json; data=json.load(open('detections.json')); [print(f\"  - confidence: {d['confidence']:.2f}, x: {d['x']:.2f}, y: {d['y']:.2f}\") for d in data if d.get('label')=='person']"
        fi
    else
        echo "❌ INVALID JSON - this is the problem!"
        echo "Raw contents:"
        cat detections.json
    fi
else
    echo "❌ detections.json NOT FOUND"
    echo "   Detection service may not be running"
fi

echo ""
echo "========================"
echo ""
echo "Testing C++ JSON parsing:"
echo ""

# Create a test C++ program to verify parsing
cat > /tmp/test_json_parse.cpp << 'EOF'
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

int main() {
    std::ifstream file("detections.json");
    if (!file.is_open()) {
        std::cerr << "❌ Cannot open detections.json\n";
        return 1;
    }
    
    try {
        json data;
        file >> data;
        
        std::cout << "✅ JSON parsed successfully\n";
        std::cout << "   Array size: " << data.size() << "\n";
        
        int people_count = 0;
        for (const auto& det : data) {
            if (det.value("label", "") == "person") {
                people_count++;
                std::cout << "   👤 Person at x=" << det.value("x", 0.0) 
                          << ", y=" << det.value("y", 0.0) << "\n";
            }
        }
        std::cout << "   Total people: " << people_count << "\n";
        
    } catch (const std::exception& e) {
        std::cerr << "❌ Parse error: " << e.what() << "\n";
        return 1;
    }
    
    return 0;
}
EOF

g++ -std=c++17 /tmp/test_json_parse.cpp -o /tmp/test_json_parse 2>/dev/null

if [ -f /tmp/test_json_parse ]; then
    /tmp/test_json_parse
    rm /tmp/test_json_parse
else
    echo "⚠️  Could not compile test program (nlohmann-json may not be installed)"
fi

echo ""
echo "========================"
echo "Recommendations:"
echo ""
echo "1. If detections.json doesn't exist:"
echo "   → Start detection service: python src/python/detection_service.py --fps 10"
echo ""
echo "2. If JSON is invalid:"
echo "   → Check for write errors in detection service output"
echo ""
echo "3. If people count is 0 but YOLO sees people:"
echo "   → Position yourself 3-6 feet from camera"
echo "   → Ensure good lighting"
echo ""
echo "4. If C++ can't parse JSON:"
echo "   → Reinstall: sudo apt install nlohmann-json3-dev"