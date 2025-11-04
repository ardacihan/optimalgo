#include "RectangleFittingProblem.h"
#include <algorithm>
#include <complex>
#include <map>
#include <iostream>
#include <climits>

int RectangleFittingProblem::objective2(const std::vector<RectanglePlacement>& current_solution) {
    const int BIG = 1'000'000;
    const int PENALTY = 10'000;
    const int TOUCH_BONUS = 500;
    const int SPARSE_BOX_PENALTY = 50'000;
    const int FRAGMENTATION_PENALTY = 100;

    std::unordered_set<int> boxes;
    for (auto& r : current_solution) boxes.insert(r.box_id);
    int num_boxes = boxes.size();

    long long overlap = 0, touch_bonus = 0;
    for (size_t i = 0; i < current_solution.size(); ++i) {
        for (size_t j = i + 1; j < current_solution.size(); ++j) {
            if (current_solution[i].box_id != current_solution[j].box_id) continue;
            overlap += current_solution[i].getOverlapArea(current_solution[j]);
            if (edges_touching(current_solution[i], current_solution[j])) touch_bonus++;
        }
    }

    std::unordered_map<int, long long> cov;
    std::unordered_map<int, int> rect_count;
    long long unused = 0;

    for (auto& r : current_solution) {
        long long area = r.get_actual_width() * r.get_actual_height();
        cov[r.box_id] += area;
        rect_count[r.box_id]++;
    }

    long long sparse_penalty = 0;
    for (int b : boxes) {
        long long box_area = 1LL * L * L;
        long long used_area = cov[b];
        unused += (box_area - used_area);

        if (used_area * 100 < box_area * 30) {
            sparse_penalty += SPARSE_BOX_PENALTY;
        }

        if (rect_count[b] <= 2) {
            sparse_penalty += SPARSE_BOX_PENALTY / 2;
        }
    }

    long long fragmentation = 0;
    for (const auto& [box_id, rects_in_box] : rect_count) {
        if (rects_in_box == 0) continue;

        int min_x = L, max_x = 0, min_y = L, max_y = 0;
        for (const auto& r : current_solution) {
            if (r.box_id != box_id) continue;
            min_x = std::min(min_x, r.x);
            max_x = std::max(max_x, r.x + r.get_actual_width());
            min_y = std::min(min_y, r.y);
            max_y = std::max(max_y, r.y + r.get_actual_height());
        }

        long long bbox_area = (long long)(max_x - min_x) * (max_y - min_y);
        long long actual_coverage = cov[box_id];

        if (bbox_area > actual_coverage * 2) {
            fragmentation += (bbox_area - actual_coverage) / 10;
        }
    }

    long long score = -(long long)num_boxes * BIG;
    score -= overlap * PENALTY;
    score -= unused / 100;
    score += touch_bonus * TOUCH_BONUS;
    score -= sparse_penalty;
    score -= fragmentation * FRAGMENTATION_PENALTY;

    if (score > INT_MAX) return INT_MAX;
    if (score < INT_MIN) return INT_MIN;
    return (int)score;
}



int RectangleFittingProblem::objective(const std::vector<RectanglePlacement>& current_solution) {
    // ------------------------------------------------------------------
    // YENİ PARAMETRE: Kenara dokunma uzunluğunu ödüllendirmek için
    const int edge_touch_length_reward_per_unit = 5;
    // Temel ödüllere göre güçlü bir teşvik için daha yüksek bir değer seçildi (touching_reward=10)
    // ------------------------------------------------------------------

    const int box_cost = -1000000;
    const int scaling_per_box_reward = 100;
    const int touching_reward = 10; // Dikdörtgen-dikdörtgen dokunma ödülü

    // Olası büyük puanlarla başa çıkmak için long long kullanın
    long long score = 0;

    std::unordered_set<int> boxes;
    std::unordered_map<int, int> box_rect_count;

    for (auto& r : current_solution) {
        boxes.insert(r.box_id);
        box_rect_count[r.box_id]++;
    }

    score += (long long)box_cost * boxes.size();

    for (auto& box_pair : box_rect_count) {
        int count = box_pair.second;
        score += (long long)count * count * scaling_per_box_reward;
    }

    // Dikdörtgen-dikdörtgen dokunma ödülü
    for (auto& r : current_solution) {
        for (auto& r2 : current_solution) {
            if (r.box_id == r2.box_id) {
                // Not: Orijinal kodunuzda tüm çiftler (i,j) ve (j,i) kontrol ediliyor, bu da puanı ikiye katlar.
                // Bu davranışı korumak için döngüyü olduğu gibi bıraktım.
                if (edges_touching(r, r2)) {
                    score += touching_reward;
                }
            }
        }
    }

    // ------------------------------------------------------------------
    // YENİ: KENARA DOKUNAN UZUNLUK ÖDÜLÜ
    for (auto& r : current_solution) {
        int touch_length = 0;
        int rect_w = r.get_actual_width();
        int rect_h = r.get_actual_height();

        // 1. Sol Kenar (x = 0)
        if (r.x == 0) {
            touch_length += rect_h;
        }

        // 2. Alt Kenar (y = 0)
        if (r.y == 0) {
            touch_length += rect_w;
        }

        // 3. Sağ Kenar (x + width = L)
        if (r.x + rect_w == L) {
            touch_length += rect_h;
        }

        // 4. Üst Kenar (y + height = L)
        if (r.y + rect_h == L) {
            touch_length += rect_w;
        }

        // Toplam dokunma uzunluğunu birim başına ödülle çarparak puana ekleyin
        score += (long long)touch_length * edge_touch_length_reward_per_unit;
    }
    // ------------------------------------------------------------------


    // Sonuç dönüşü için long long'dan int'e güvenli dönüş
    if (score > INT_MAX) return INT_MAX;
    if (score < INT_MIN) return INT_MIN;
    return (int)score;
}


bool RectangleFittingProblem::check_no_overlaps(const std::vector<RectanglePlacement>& current_solution) const {
    for (size_t i = 0; i < current_solution.size(); ++i) {
        for (size_t j = i + 1; j < current_solution.size(); ++j) {
            const auto& a = current_solution[i];
            const auto& b = current_solution[j];
            if (a.box_id == b.box_id &&
                a.x < b.x + b.get_actual_width() &&
                a.x + a.get_actual_width() > b.x &&
                a.y < b.y + b.get_actual_height() &&
                a.y + a.get_actual_height() > b.y) {
                return false;
            }
        }
    }
    return true;
}

bool RectangleFittingProblem::check_within_boxes(const std::vector<RectanglePlacement>& current_solution) const {
    for (const auto& rect : current_solution) {
        double right = rect.x + rect.get_actual_width();
        double bottom = rect.y + rect.get_actual_height();

        if (rect.x < 0 || rect.y < 0 || right > L || bottom > L) {
            std::cout << "Rectangle out of bounds: (" << rect.x << ", " << rect.y
                      << ") to (" << right << ", " << bottom << ") in box [0,0] to ["
                      << L << "," << L << "]\n";
            return false;
        }
    }
    return true;
}

bool RectangleFittingProblem::solution_legal(const std::vector<RectanglePlacement>& current_solution) const {
    return check_no_overlaps(current_solution) && check_within_boxes(current_solution);
}

bool RectangleFittingProblem::edges_touching(const RectanglePlacement& r1, const RectanglePlacement& r2) {
    if (r1.collides(r2)) return false;

    int w1 = r1.get_actual_width(), h1 = r1.get_actual_height();
    int w2 = r2.get_actual_width(), h2 = r2.get_actual_height();

    bool vertical = (r1.x + w1 == r2.x || r2.x + w2 == r1.x) &&
                    !(r1.y + h1 <= r2.y || r2.y + h2 <= r1.y);
    bool horizontal = (r1.y + h1 == r2.y || r2.y + h2 == r1.y) &&
                      !(r1.x + w1 <= r2.x || r2.x + w2 <= r1.x);

    return vertical || horizontal;
}