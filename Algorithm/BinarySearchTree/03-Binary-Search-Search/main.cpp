#include <iostream>
#include <vector>
#include <string>
#include <ctime>
#include "SequenceST.h"
#include "FileOps.h"

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
};


// ���Զ�����������˳����ұ�֮������ܲ��
// ����������������ԶԶ����˳����ұ�
int main() {

	// ʹ��ʥ����Ϊ���ǵĲ�������
	string filename = "bible.txt";
	vector<string> words;
	if (FileOps::readFile(filename, words)) {

		cout << "There are totally " << words.size() << " words in " << filename << endl;
		cout << endl;


		// ���� BST
		time_t startTime = clock();

		// ͳ��ʥ�������дʵĴ�Ƶ
		// ע: �����Ƶͳ�Ʒ���Լ�ª, û�п��Ǻܶ��ı������е���������
		// ������ֻ�����ܲ�����
		BST<string, int> bst = BST<string, int>();
		for (vector<string>::iterator iter = words.begin(); iter != words.end(); iter++) {
			int* res = bst.search(*iter);
			if (res == NULL)
				bst.insert(*iter, 1);
			else
				(*res)++;
		}

		// ���ʥ����godһ�ʳ��ֵ�Ƶ��
		if (bst.contain("god"))
			cout << "'god' : " << *bst.search("god") << endl;
		else
			cout << "No word 'god' in " << filename << endl;

		time_t endTime = clock();

		cout << "BST , time: " << double(endTime - startTime) / CLOCKS_PER_SEC << " s." << endl;
		cout << endl;


		// ����˳����ұ� SST
		startTime = clock();

		// ͳ��ʥ�������дʵĴ�Ƶ
		// ע: �����Ƶͳ�Ʒ���Լ�ª, û�п��Ǻܶ��ı������е���������
		// ������ֻ�����ܲ�����
		SequenceST<string, int> sst = SequenceST<string, int>();
		for (vector<string>::iterator iter = words.begin(); iter != words.end(); iter++) {
			int* res = sst.search(*iter);
			if (res == NULL)
				sst.insert(*iter, 1);
			else
				(*res)++;
		}

		// ���ʥ����godһ�ʳ��ֵ�Ƶ��
		if (sst.contain("god"))
			cout << "'god' : " << *sst.search("god") << endl;
		else
			cout << "No word 'god' in " << filename << endl;

		endTime = clock();

		cout << "SST , time: " << double(endTime - startTime) / CLOCKS_PER_SEC << " s." << endl;
	}

	return 0;
}