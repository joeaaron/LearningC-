#include <iostream>
#include <queue>
#include <cassert>
#include <ctime>

using namespace std;

// ����������
template <typename Key, typename Value>
class BST {

private:
	// ���еĽڵ�Ϊ˽�еĽṹ��, ��粻��Ҫ�˽�����������ڵ�ľ���ʵ��
	struct Node {
		Key key;
		Value value;
		Node* left;
		Node* right;

		Node(Key key, Value value) {
			this->key = key;
			this->value = value;
			this->left = this->right = NULL;
		}
	};

	Node* root; // ���ڵ�
	int count;  // ���еĽڵ����

public:
	// ���캯��, Ĭ�Ϲ���һ�ÿն���������
	BST() {
		root = NULL;
		count = 0;
	}

	// ��������, �ͷŶ��������������пռ�
	~BST() {
		destroy(root);
	}

	// ���ض����������Ľڵ����
	int size() {
		return count;
	}

	// ���ض����������Ƿ�Ϊ��
	bool isEmpty() {
		return count == 0;
	}

	// ������������в���һ���µ�(key, value)���ݶ�
	void insert(Key key, Value value) {
		root = insert(root, key, value);
	}

	// �鿴�������������Ƿ���ڼ�key
	bool contain(Key key) {
		return contain(root, key);
	}

	// �ڶ�����������������key����Ӧ��ֵ��������ֵ������, �򷵻�NULL
	Value* search(Key key) {
		return search(root, key);
	}

	// ������������ǰ�����
	void preOrder() {
		preOrder(root);
	}

	// �������������������
	void inOrder() {
		inOrder(root);
	}

	// �����������ĺ������
	void postOrder() {
		postOrder(root);
	}

	// �����������Ĳ������
	void levelOrder() {

		queue<Node*> q;
		q.push(root);
		while (!q.empty()) {

			Node* node = q.front();
			q.pop();

			cout << node->key << endl;

			if (node->left)
				q.push(node->left);
			if (node->right)
				q.push(node->right);
		}
	}

	// Ѱ�Ҷ�������������С�ļ�ֵ
	Key minimum() {
		assert(count != 0);
		Node* minNode = minimum(root);
		return minNode->key;
	}

	// Ѱ�Ҷ��������������ļ�ֵ
	Key maximum() {
		assert(count != 0);
		Node* maxNode = maximum(root);
		return maxNode->key;
	}

	// �Ӷ�����������ɾ����Сֵ���ڽڵ�
	void removeMin() {
		if (root)
			root = removeMin(root);
	}

	// �Ӷ�����������ɾ�����ֵ���ڽڵ�
	void removeMax() {
		if (root)
			root = removeMax(root);
	}

private:
	// ����nodeΪ���Ķ�����������, ����ڵ�(key, value), ʹ�õݹ��㷨
	// ���ز����½ڵ��Ķ����������ĸ�
	Node* insert(Node* node, Key key, Value value) {

		if (node == NULL) {
			count++;
			return new Node(key, value);
		}

		if (key == node->key)
			node->value = value;
		else if (key < node->key)
			node->left = insert(node->left, key, value);
		else    // key > node->key
			node->right = insert(node->right, key, value);

		return node;
	}

	// �鿴��nodeΪ���Ķ������������Ƿ������ֵΪkey�Ľڵ�, ʹ�õݹ��㷨
	bool contain(Node* node, Key key) {

		if (node == NULL)
			return false;

		if (key == node->key)
			return true;
		else if (key < node->key)
			return contain(node->left, key);
		else // key > node->key
			return contain(node->right, key);
	}

	// ����nodeΪ���Ķ����������в���key����Ӧ��value, �ݹ��㷨
	// ��value������, �򷵻�NULL
	Value* search(Node* node, Key key) {

		if (node == NULL)
			return NULL;

		if (key == node->key)
			return &(node->value);
		else if (key < node->key)
			return search(node->left, key);
		else // key > node->key
			return search(node->right, key);
	}

	// ����nodeΪ���Ķ�������������ǰ�����, �ݹ��㷨
	void preOrder(Node* node) {

		if (node != NULL) {
			cout << node->key << endl;
			preOrder(node->left);
			preOrder(node->right);
		}
	}

	// ����nodeΪ���Ķ��������������������, �ݹ��㷨
	void inOrder(Node* node) {

		if (node != NULL) {
			inOrder(node->left);
			cout << node->key << endl;
			inOrder(node->right);
		}
	}

	// ����nodeΪ���Ķ������������к������, �ݹ��㷨
	void postOrder(Node* node) {

		if (node != NULL) {
			postOrder(node->left);
			postOrder(node->right);
			cout << node->key << endl;
		}
	}

	// �ͷ���nodeΪ���Ķ��������������нڵ�
	// ���ú��������ĵݹ��㷨
	void destroy(Node* node) {

		if (node != NULL) {
			destroy(node->left);
			destroy(node->right);

			delete node;
			count--;
		}
	}

	// ������nodeΪ���Ķ�������������С��ֵ���ڵĽڵ�
	Node* minimum(Node* node) {
		if (node->left == NULL)
			return node;

		return minimum(node->left);
	}

	// ������nodeΪ���Ķ���������������ֵ���ڵĽڵ�
	Node* maximum(Node* node) {
		if (node->right == NULL)
			return node;

		return maximum(node->right);
	}

	// ɾ������nodeΪ���Ķ����������е���С�ڵ�
	// ����ɾ���ڵ���µĶ����������ĸ�
	Node* removeMin(Node* node) {

		if (node->left == NULL) {

			Node* rightNode = node->right;
			delete node;
			count--;
			return rightNode;
		}

		node->left = removeMin(node->left);
		return node;
	}

	// ɾ������nodeΪ���Ķ����������е����ڵ�
	// ����ɾ���ڵ���µĶ����������ĸ�
	Node* removeMax(Node* node) {

		if (node->right == NULL) {

			Node* leftNode = node->left;
			delete node;
			count--;
			return leftNode;
		}

		node->right = removeMax(node->right);
		return node;
	}
};


// ���Զ����������е�removeMin��removeMax
int main() {

	srand(time(NULL));
	BST<int, int> bst = BST<int, int>();

	// ȡn��ȡֵ��Χ��[0...m)����������Ž�������������
	int n = 100;
	int m = 100;
	for (int i = 0; i < n; i++) {
		int key = rand() % m;
		// Ϊ�˺������Է���,����valueֵȡ��keyֵһ��
		int value = key;
		//cout<<key<<" ";
		bst.insert(key, value);
	}
	// ע��, ����������ɵ��������ظ�, ����bst�е����������������С��n��

	// ���� removeMin
	// �����Ԫ��Ӧ���Ǵ�С�������е�
	cout << "Test removeMin: " << endl;
	while (!bst.isEmpty()) {
		cout << "min: " << bst.minimum() << " , ";
		bst.removeMin();
		cout << "After removeMin, size = " << bst.size() << endl;
	}
	cout << endl;


	for (int i = 0; i < n; i++) {
		int key = rand() % n;
		// Ϊ�˺������Է���,����valueֵȡ��keyֵһ��
		int value = key;
		//cout<<key<<" ";
		bst.insert(key, value);
	}
	// ע��, ����������ɵ��������ظ�, ����bst�е����������������С��n��

	// ���� removeMax
	// �����Ԫ��Ӧ���ǴӴ�С���е�
	cout << "Test removeMax: " << endl;
	while (!bst.isEmpty()) {
		cout << "max: " << bst.maximum() << " , ";
		bst.removeMax();
		cout << "After removeMax, size = " << bst.size() << endl;
	}

	return 0;
}