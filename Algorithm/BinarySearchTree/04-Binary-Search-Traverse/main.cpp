#include <iostream>
#include <queue>
#include <ctime>

using namespace std;

template<typename Key, typename Value>
class BST {
private:
	// �����������еĽڵ�Ϊ˽�еĽṹ�壬��粻��Ҫ�˽�����������ڵ�ľ���ʵ��
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

	Node* root;      //���ڵ�
	int count;       //�ڵ����

public:
	//���캯����Ĭ�Ϲ���һ�ÿն���������
	BST() {
		root = NULL;
		count = 0;
	}
	~BST() {

	}
	//���ض����������Ľڵ����
	int size() {
		return count;
	}
	//���ض����������Ƿ�Ϊ��
	bool isEmpty() {
		return count == 0;
	}
	//������������в���һ���µģ�key,value)���ݶ�
	void insert(Key key, Value value) {
		root = insert(root, key, value);
	}
	//�鿴�������������Ƿ���ڼ�key
	bool contain(Key key) {
		return contain(root, key);
	}
	//�ڶ�����������������key����Ӧ��ֵ��������ֵ�����ڣ��򷵻�NULL
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
private:
	Node* insert(Node* node, Key key, Value value) {
		if (node == NULL) {
			count ++;
			return new Node(key, value);
		}
		
		if (key == node->key)
			node->value = value;
		else if (key < node->key)
			node->left = insert(node->left, key, value);
		else
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
		else
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
};

// ���Զ�����������ǰ�к������
int main() {

	srand(time(NULL));
	BST<int, int> bst = BST<int, int>();

	// ȡn��ȡֵ��Χ��[0...m)����������Ž�������������
	int N = 10;
	int M = 100;
	for (int i = 0; i < N; i++) {
		int key = rand() % M;
		// Ϊ�˺������Է���,����valueֵȡ��keyֵһ��
		int value = key;
		cout << key << " ";
		bst.insert(key, value);
	}
	cout << endl;

	// ���Զ�����������size()
	cout << "size: " << bst.size() << endl << endl;

	// ���Զ�����������ǰ����� preOrder
	cout << "preOrder: " << endl;
	bst.preOrder();
	cout << endl;

	// ���Զ������������������ inOrder
	cout << "inOrder: " << endl;
	bst.inOrder();
	cout << endl;

	// ���Զ����������ĺ������ postOrder
	cout << "postOrder: " << endl;
	bst.postOrder();
	cout << endl;

	return 0;
}