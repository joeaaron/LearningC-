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
	//������������в���һ���µ�(key, value)���ݶ�
	void insert(Key key, Value value) {
		root = insert(root, key, value);
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
};

int main() {
	return 0;
}