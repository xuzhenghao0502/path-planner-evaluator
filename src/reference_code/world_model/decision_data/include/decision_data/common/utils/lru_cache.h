/**
 * @file lru_cache.h
 * @brief 此文件定义了一个模板类 `LRUCache`，用于实现最近最少使用（LRU）缓存机制。
 * @details LRU 缓存是一种缓存淘汰策略，当缓存满时，会优先淘汰最近最少使用的数据。该类使用双向链表和哈希表实现，
 * 保证插入、查找和删除操作的时间复杂度为 O(1)。
 */

#pragma once

#include <iostream>
#include <map>
#include <mutex>
#include <utility>
#include <vector>

namespace gpal {
namespace pnc {
namespace prediction {
namespace common {

/**
 * @struct Node
 * @brief 双向链表节点的模板结构体，用于 `LRUCache` 中存储键值对。
 * @tparam K 键的类型。
 * @tparam V 值的类型。
 */
template <class K, class V>
struct Node {
  K key;       ///< 节点的键。
  V val;       ///< 节点的值。
  Node* prev;  ///< 指向前一个节点的指针。
  Node* next;  ///< 指向后一个节点的指针。

  /**
   * @brief 默认构造函数。
   * @details 初始化节点的前后指针为 `nullptr`。
   */
  Node() : key(), val(), prev(nullptr), next(nullptr) {}

  /**
   * @brief 带参数的构造函数。
   * @details 使用给定的键和值初始化节点，并将前后指针置为 `nullptr`。
   * @tparam VV 值的完美转发类型。
   * @param[in] _key 节点的键。
   * @param[in] _val 节点的值，使用完美转发。
   */
  template <typename VV>
  Node(const K& _key, VV&& _val) : key(_key), val(std::forward<VV>(_val)), prev(nullptr), next(nullptr) {}
};

/**
 * @class LRUCache
 * @brief 实现最近最少使用（LRU）缓存机制的模板类。
 * @tparam K 键的类型。
 * @tparam V 值的类型。
 */
template <class K, class V>
class LRUCache {
 public:
  static constexpr size_t kDefaultCapacity = 10;  ///< 默认缓存容量。

  /**
   * @brief 构造函数。
   * @details 初始化缓存的容量，并调用 `Init` 方法初始化双向链表和计数器。
   * @param[in] capacity 缓存的容量，默认为 `kDefaultCapacity`。
   */
  explicit LRUCache(const size_t capacity = kDefaultCapacity) : capacity_(capacity), head_(), tail_() { Init(); }

  /**
   * @brief 析构函数。
   * @details 调用 `Clear` 方法清空缓存。
   */
  ~LRUCache() { Clear(); }

  /**
   * @brief 获取缓存内部的映射表的常量引用。
   * @return const std::map<K, Node<K, V>>& 缓存内部映射表的常量引用。
   */
  const std::map<K, Node<K, V>>& GetMap() const { return map_; }

  /**
   * @brief 将缓存中的键值对复制到指定的映射表中。
   * @details 遍历缓存内部的映射表，将键值对插入到传入的映射表中。
   * @param[out] cache 用于存储缓存键值对的映射表指针。
   *
   * @par 流程图:
   * @startuml
   * start
   * :初始化迭代器指向 map_ 起始位置;
   * while (迭代器未到 map_ 末尾?) is (否)
   *   stop
   * while (迭代器未到 map_ 末尾?) is (是)
   *   :将当前键值对插入到 cache 中;
   *   :迭代器指向下一个元素;
   * endwhile
   * stop
   * @enduml
   */
  void GetCache(std::map<K, V>* cache) {
    for (auto it = map_.begin(); it != map_.end(); ++it) {
      cache->emplace(it->first, it->second.val);
    }
  }

  /**
   * @brief 重载 `[]` 运算符，用于通过键获取或插入值。
   * @details 如果键不存在且缓存已满，会先淘汰最近最少使用的元素。
   * @param[in] key 要查找或插入的键。
   * @return V& 键对应的值的引用。
   *
   * @par 流程图:
   * @startuml
   * start
   * :检查键是否存在于缓存中;
   * if (键不存在?) then (是)
   *   :检查缓存是否已满;
   *   if (缓存已满?) then (是)
   *     :获取并淘汰最近最少使用的元素;
   *   endif
   * endif
   * :返回键对应的值的引用;
   * stop
   * @enduml
   */
  V& operator[](const K& key) {
    if (!Contains(key)) {
      K obsolete;
      GetObsolete(&obsolete);
    }
    return map_[key].val;
  }

  /**
   * @brief 静默地将缓存中所有值的指针添加到指定的向量中。
   * @details 遍历缓存内部的映射表，将每个节点的值的指针添加到传入的向量中。
   * @param[out] ret 用于存储值指针的向量指针。
   *
   * @par 流程图:
   * @startuml
   * start
   * :初始化迭代器指向 map_ 起始位置;
   * while (迭代器未到 map_ 末尾?) is (否)
   *   stop
   * while (迭代器未到 map_ 末尾?) is (是)
   *   :将当前节点值的指针添加到 ret 中;
   *   :迭代器指向下一个元素;
   * endwhile
   * stop
   * @enduml
   */
  void GetAllSilently(std::vector<V*>* ret) {
    for (auto it = map_.begin(); it != map_.end(); ++it) {
      ret->push_back(&it->second.val);
    }
  }

  /**
   * @brief 插入或更新缓存中的键值对。
   * @details 如果键已存在，更新其值并将该节点移到链表头部；如果键不存在且缓存已满，先淘汰最近最少使用的元素。
   * @tparam VV 值的完美转发类型。
   * @param[in] key 要插入或更新的键。
   * @param[in] val 要插入或更新的值，使用完美转发。
   * @return bool 插入或更新成功返回 `true`，否则返回 `false`。
   */
  template <typename VV>
  bool Put(const K& key, VV&& val) {
    K tmp;
    return Update(key, std::forward<VV>(val), &tmp, false, false);
  }

  /**
   * @brief 仅更新缓存中已存在的键值对。
   * @details 如果键存在，更新其值并将该节点移到链表头部；如果键不存在，返回 `false`。
   * @tparam VV 值的完美转发类型。
   * @param[in] key 要更新的键。
   * @param[in] val 要更新的值，使用完美转发。
   * @return bool 更新成功返回 `true`，否则返回 `false`。
   */
  template <typename VV>
  bool Update(const K& key, VV&& val) {
    if (!Contains(key)) {
      return false;
    }
    K tmp;
    return Update(key, std::forward<VV>(val), &tmp, true, false);
  }

  /**
   * @brief 静默地更新缓存中已存在的键值对。
   * @details 如果键存在，更新其值但不调整节点在链表中的位置；如果键不存在，返回 `false`。
   * @tparam VV 值的完美转发类型。
   * @param[in] key 要更新的键。
   * @param[in] val 指向要更新的值的指针，使用完美转发。
   * @return bool 更新成功返回 `true`，否则返回 `false`。
   */
  template <typename VV>
  bool UpdateSilently(const K& key, VV* val) {
    if (!Contains(key)) {
      return false;
    }
    K tmp;
    return Update(key, std::forward<VV>(*val), &tmp, true, true);
  }

  /**
   * @brief 仅插入新的键值对。
   * @details 如果键不存在，插入新的键值对并将该节点移到链表头部；如果键已存在，返回 `false`。
   * @tparam VV 值的完美转发类型。
   * @param[in] key 要插入的键。
   * @param[in] val 指向要插入的值的指针，使用完美转发。
   * @return bool 插入成功返回 `true`，否则返回 `false`。
   */
  template <typename VV>
  bool Add(const K& key, VV* val) {
    K tmp;
    return Update(key, std::forward<VV>(*val), &tmp, true, false);
  }

  /**
   * @brief 插入或更新键值对，并获取被淘汰的键。
   * @details 如果键已存在，更新其值并将该节点移到链表头部；如果键不存在且缓存已满，先淘汰最近最少使用的元素并返回其键。
   * @tparam VV 值的完美转发类型。
   * @param[in] key 要插入或更新的键。
   * @param[in] val 指向要插入或更新的值的指针，使用完美转发。
   * @param[out] obs 用于存储被淘汰的键的指针。
   * @return bool 插入或更新成功返回 `true`，否则返回 `false`。
   */
  template <typename VV>
  bool PutAndGetObsolete(const K& key, VV* val, K* obs) {
    return Update(key, std::forward<VV>(*val), obs, false, false);
  }

  /**
   * @brief 仅插入新的键值对，并获取被淘汰的键。
   * @details 如果键不存在，插入新的键值对并将该节点移到链表头部；如果键已存在，返回
   * `false`；如果缓存已满，先淘汰最近最少使用的元素并返回其键。
   * @tparam VV 值的完美转发类型。
   * @param[in] key 要插入的键。
   * @param[in] val 指向要插入的值的指针，使用完美转发。
   * @param[out] obs 用于存储被淘汰的键的指针。
   * @return bool 插入成功返回 `true`，否则返回 `false`。
   */
  template <typename VV>
  bool AddAndGetObsolete(const K& key, VV* val, K* obs) {
    return Update(key, std::forward<VV>(*val), obs, true, false);
  }

  /**
   * @brief 静默地获取指定键对应的值的指针。
   * @details 如果键存在，返回其值的指针但不调整节点在链表中的位置；如果键不存在，返回 `nullptr`。
   * @param[in] key 要查找的键。
   * @return V* 键对应的值的指针，若键不存在则返回 `nullptr`。
   */
  V* GetSilently(const K& key) { return Get(key, true); }

  /**
   * @brief 获取指定键对应的值的指针。
   * @details 如果键存在，返回其值的指针并将该节点移到链表头部；如果键不存在，返回 `nullptr`。
   * @param[in] key 要查找的键。
   * @return V* 键对应的值的指针，若键不存在则返回 `nullptr`。
   */
  V* Get(const K& key) { return Get(key, false); }

  /**
   * @brief 静默地将指定键对应的值复制到给定的指针中。
   * @details 如果键存在，将其值复制到传入的指针中但不调整节点在链表中的位置；如果键不存在，返回 `false`。
   * @param[in] key 要查找的键。
   * @param[out] val 用于存储值的指针。
   * @return bool 复制成功返回 `true`，否则返回 `false`。
   */
  bool GetCopySilently(const K& key, V* const val) { return GetCopy(key, val, true); }

  /**
   * @brief 将指定键对应的值复制到给定的指针中。
   * @details 如果键存在，将其值复制到传入的指针中并将该节点移到链表头部；如果键不存在，返回 `false`。
   * @param[in] key 要查找的键。
   * @param[out] val 用于存储值的指针。
   * @return bool 复制成功返回 `true`，否则返回 `false`。
   */
  bool GetCopy(const K& key, V* const val) { return GetCopy(key, val, false); }

  /**
   * @brief 获取当前缓存中元素的数量。
   * @return size_t 当前缓存中元素的数量。
   */
  size_t size() { return size_; }

  /**
   * @brief 判断缓存是否已满。
   * @return bool 缓存已满返回 `true`，否则返回 `false`。
   */
  bool Full() { return size() > 0 && size() >= capacity_; }

  /**
   * @brief 判断缓存是否为空。
   * @return bool 缓存为空返回 `true`，否则返回 `false`。
   */
  bool Empty() { return size() == 0; }

  /**
   * @brief 获取缓存的容量。
   * @return size_t 缓存的容量。
   */
  size_t capacity() { return capacity_; }

  /**
   * @brief 获取双向链表的第一个节点。
   * @return Node<K, V>* 双向链表的第一个节点指针，若链表为空则返回 `nullptr`。
   */
  Node<K, V>* First() {
    if (size()) {
      return head_.next;
    }
    return nullptr;
  }

  /**
   * @brief 获取双向链表的最后一个节点。
   * @return Node<K, V>* 双向链表的最后一个节点指针，若链表为空则返回 `nullptr`。
   */
  Node<K, V>* Last() {
    if (size()) {
      return tail_.prev;
    }
    return nullptr;
  }

  /**
   * @brief 判断缓存中是否包含指定的键。
   * @param[in] key 要查找的键。
   * @return bool 包含该键返回 `true`，否则返回 `false`。
   */
  bool Contains(const K& key) { return map_.find(key) != map_.end(); }

  /**
   * @brief 将指定键对应的节点移到链表头部。
   * @details 如果键存在，将该节点从链表中分离出来，再插入到链表头部。
   * @param[in] key 要提升优先级的键。
   * @return bool 操作成功返回 `true`，否则返回 `false`。
   *
   * @par 流程图:
   * @startuml
   * start
   * :检查键是否存在于缓存中;
   * if (键存在?) then (是)
   *   :获取该键对应的节点指针;
   *   :将该节点从链表中分离;
   *   :将该节点插入到链表头部;
   *   :返回 true;
   * else (否)
   *   :返回 false;
   * endif
   * stop
   * @enduml
   */
  bool Prioritize(const K& key) {
    if (Contains(key)) {
      auto* node = &map_[key];
      Detach(node);
      Attach(node);
      return true;
    }
    return false;
  }

  /**
   * @brief 清空缓存。
   * @details 清空内部的映射表，并调用 `Init` 方法重新初始化双向链表和计数器。
   *
   * @par 流程图:
   * @startuml
   * start
   * :清空 map_;
   * :调用 Init 方法重新初始化;
   * stop
   * @enduml
   */
  void Clear() {
    map_.clear();
    Init();
  }

  /**
   * @brief 从缓存中移除指定键对应的元素。
   * @details 如果键存在，将该节点从链表中分离出来，并从映射表中删除该键。
   * @param[in] key 要移除的键。
   * @return bool 移除成功返回 `true`，否则返回 `false`。
   *
   * @par 流程图:
   * @startuml
   * start
   * :检查键是否存在于缓存中;
   * if (键存在?) then (是)
   *   :获取该键对应的节点指针;
   *   :将该节点从链表中分离;
   *   :从 map_ 中删除该键;
   *   :返回 true;
   * else (否)
   *   :返回 false;
   * endif
   * stop
   * @enduml
   */
  bool Remove(const K& key) {
    if (!Contains(key)) {
      return false;
    }
    auto* node = &map_[key];
    Detach(node);
    map_.erase(key);
    return true;
  }

  /**
   * @brief 更改缓存的容量。
   * @details 如果当前缓存中的元素数量大于新的容量，返回 `false`；否则更新缓存容量。
   * @param[in] capacity 新的缓存容量。
   * @return bool 更改成功返回 `true`，否则返回 `false`。
   */
  bool ChangeCapacity(const size_t capacity) {
    if (size() > capacity) {
      return false;
    }
    capacity_ = capacity;
    return true;
  }

 private:
  size_t capacity_;              ///< 缓存的容量。
  size_t size_;                  ///< 当前缓存中元素的数量。
  std::map<K, Node<K, V>> map_;  ///< 用于快速查找节点的映射表。
  Node<K, V> head_;              ///< 双向链表的头节点。
  Node<K, V> tail_;              ///< 双向链表的尾节点。

  /**
   * @brief 初始化双向链表和计数器。
   * @details 设置头节点和尾节点的指针，并将元素数量置为 0，清空映射表。
   */
  void Init() {
    head_.prev = nullptr;
    head_.next = &tail_;
    tail_.prev = &head_;
    tail_.next = nullptr;
    size_ = 0;
    map_.clear();
  }

  /**
   * @brief 将节点从双向链表中分离出来。
   * @details 调整节点前后节点的指针，将该节点从链表中移除，并更新元素数量。
   * @param[in] node 要分离的节点指针。
   */
  void Detach(Node<K, V>* node) {
    if (node->prev != nullptr) {
      node->prev->next = node->next;
    }
    if (node->next != nullptr) {
      node->next->prev = node->prev;
    }
    node->prev = nullptr;
    node->next = nullptr;
    --size_;
  }

  /**
   * @brief 将节点插入到双向链表的头部。
   * @details 调整节点和头节点的指针，将该节点插入到链表头部，并更新元素数量。
   * @param[in] node 要插入的节点指针。
   */
  void Attach(Node<K, V>* node) {
    node->prev = &head_;
    node->next = head_.next;
    head_.next = node;
    if (node->next != nullptr) {
      node->next->prev = node;
    }
    ++size_;
  }

  /**
   * @brief 插入、更新键值对，并可选择获取被淘汰的键。
   * @details 根据 `add_only` 参数决定是仅插入还是插入或更新，根据 `silent_update` 参数决定是否调整节点位置。
   * @tparam VV 值的完美转发类型。
   * @param[in] key 要插入或更新的键。
   * @param[in] val 要插入或更新的值，使用完美转发。
   * @param[out] obs 用于存储被淘汰的键的指针。
   * @param[in] add_only 仅插入标志，为 `true` 时仅插入新键值对。
   * @param[in] silent_update 静默更新标志，为 `true` 时不调整节点位置。
   * @return bool 操作成功返回 `true`，否则返回 `false`。
   *
   * @par 流程图:
   * @startuml
   * start
   * :检查 obs 是否为 nullptr;
   * if (obs 为 nullptr?) then (是)
   *   :返回 false;
   * else (否)
   *   :检查键是否存在于缓存中;
   *   if (键存在?) then (是)
   *     if (add_only 为 false?) then (是)
   *       :更新该键对应的值;
   *       if (silent_update 为 false?) then (是)
   *         :将该节点从链表中分离;
   *         :将该节点插入到链表头部;
   *       else (否)
   *         :返回 false;
   *       endif
   *     endif
   *   else (否)
   *     :检查缓存是否已满;
   *     if (缓存已满?) then (是)
   *       :获取并淘汰最近最少使用的元素;
   *       if (获取失败?) then (是)
   *         :返回 false;
   *       endif
   *     endif
   *     :插入新的键值对;
   *     :将新节点插入到链表头部;
   *   endif
   *   :返回 true;
   * endif
   * stop
   * @enduml
   */
  template <typename VV>
  bool Update(const K& key, VV&& val, K* obs, bool add_only, bool silent_update) {
    if (obs == nullptr) {
      return false;
    }
    if (Contains(key)) {
      if (!add_only) {
        map_[key].val = std::forward<VV>(val);
        if (!silent_update) {
          auto* node = &map_[key];
          Detach(node);
          Attach(node);
        } else {
          return false;
        }
      }
    } else {
      if (Full() && !GetObsolete(obs)) {
        return false;
      }

      map_.emplace(key, Node<K, V>(key, std::forward<VV>(val)));
      Attach(&map_[key]);
    }
    return true;
  }

  /**
   * @brief 获取指定键对应的值的指针，并可选择是否调整节点位置。
   * @details 如果键存在，根据 `silent` 参数决定是否将该节点移到链表头部，然后返回其值的指针；如果键不存在，返回
   * `nullptr`。
   * @param[in] key 要查找的键。
   * @param[in] silent 静默标志，为 `true` 时不调整节点位置。
   * @return V* 键对应的值的指针，若键不存在则返回 `nullptr`。
   *
   * @par 流程图:
   * @startuml
   * start
   * :检查键是否存在于缓存中;
   * if (键存在?) then (是)
   *   :获取该键对应的节点指针;
   *   if (silent 为 false?) then (是)
   *     :将该节点从链表中分离;
   *     :将该节点插入到链表头部;
   *   endif
   *   :返回该节点值的指针;
   * else (否)
   *   :返回 nullptr;
   * endif
   * stop
   * @enduml
   */
  V* Get(const K& key, bool silent) {
    if (Contains(key)) {
      auto* node = &map_[key];
      if (!silent) {
        Detach(node);
        Attach(node);
      }
      return &node->val;
    }
    return nullptr;
  }

  /**
   * @brief 将指定键对应的值复制到给定的指针中，并可选择是否调整节点位置。
   * @details 如果键存在，根据 `silent`
   * 参数决定是否将该节点移到链表头部，然后将其值复制到传入的指针中；如果键不存在，返回 `false`。
   * @param[in] key 要查找的键。
   * @param[out] val 用于存储值的指针。
   * @param[in] silent 静默标志，为 `true` 时不调整节点位置。
   * @return bool 复制成功返回 `true`，否则返回 `false`。
   *
   * @par 流程图:
   * @startuml
   * start
   * :检查键是否存在于缓存中;
   * if (键存在?) then (是)
   *   :获取该键对应的节点指针;
   *   if (silent 为 false?) then (是)
   *     :将该节点从链表中分离;
   *     :将该节点插入到链表头部;
   *   endif
   *   :将该节点的值复制到 val 中;
   *   :返回 true;
   * else (否)
   *   :返回 false;
   * endif
   * stop
   * @enduml
   */
  bool GetCopy(const K& key, V* const val, bool silent) {
    if (Contains(key)) {
      auto* node = &map_[key];
      if (!silent) {
        Detach(node);
        Attach(node);
      }
      *val = node->val;
      return true;
    }
    return false;
  }

  /**
   * @brief 获取并淘汰最近最少使用的元素。
   * @details 如果缓存已满，将双向链表的最后一个节点从链表中分离出来，记录其键，从映射表中删除该键。
   * @param[out] key 用于存储被淘汰元素的键的指针。
   * @return bool 成功淘汰返回 `true`，否则返回 `false`。
   *
   * @par 流程图:
   * @startuml
   * start
   * :检查缓存是否已满;
   * if (缓存已满?) then (是)
   *   :获取双向链表的最后一个节点指针;
   *   :将该节点从链表中分离;
   *   :将该节点的键存储到 key 中;
   *   :从 map_ 中删除该键;
   *   :返回 true;
   * else (否)
   *   :返回 false;
   * endif
   * stop
   * @enduml
   */
  bool GetObsolete(K* key) {
    if (Full()) {
      auto* node = tail_.prev;
      Detach(node);
      *key = node->key;
      map_.erase(node->key);
      return true;
    }
    return false;
  }
};

}  // namespace common
}  // namespace prediction
}  // namespace pnc
}  // namespace gpal