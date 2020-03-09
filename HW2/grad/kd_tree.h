#include <iostream>
#include <stdio.h>
#include <cmath>

using namespace std;

const int k = 5; // 5 dim for now
// structure to represent node of kd tree



struct Node{
	//int k;
	double point[k];
	Node *left, *right;
};

// structure to create new node of kd tree
struct Node* newNode(double arr[]){  // why we are returning node pointer here
	//int k = k;
	struct Node* temp = new Node;
	for(int i =0; i<k;i++){
		temp->point[i] = arr[i];
	}
	temp->left = temp->right = NULL;
	return temp;
};

// insert new nodes and return root of modified tree
// parameter depth is used to decide axis of comparision
Node *insertRec(Node *root,double point[],unsigned int depth){ // why the function is pointer here
	if(root == NULL){
		return newNode(point);
	}
	// calculate the current dimention of tree
	unsigned int cd = depth % k;

	if(point[cd]<(root->point[cd])){
		root->left = insertRec(root->left,point,depth+1);
	}else{
		root->right = insertRec(root->right,point,depth+1);
	}
	return root;
}
 // insert a point in kd tree
 Node* insert(Node *root,double point[])
 {
 	return insertRec(root,point,0);
 }
bool arePointsSame(double point1[], double point2[]) 
{ 
    // Compare individual pointinate values 
    for (int i = 0; i < k; ++i) 
        if (point1[i] != point2[i]) 
            return false; 
  
    return true; 
} 

bool searchRec(Node* root, double point[], unsigned depth) 
{ 
    // Base cases 
    if (root == NULL) 
        return false; 
    if (arePointsSame(root->point, point)) 
        return true; 
  
    // Current dimension is computed using current depth and total 
    // dimensions (k) 
    unsigned cd = depth % k; 
  
    // Compare point with root with respect to cd (Current dimension) 
    if (point[cd] < root->point[cd]) 
        return searchRec(root->left, point, depth + 1); 
  
    return searchRec(root->right, point, depth + 1); 
} 
  
// Searches a Point in the K D tree. It mainly uses 
// searchRec() 
bool search(Node* root, double point[]) 
{ 
    // Pass current depth as 0 
    return searchRec(root, point, 0); 
} 
Node *minNode(Node *x, Node *y, Node *z, int d) 
{ 
    Node *res = x; 
    if (y != NULL && y->point[d] < res->point[d]) 
       res = y; 
    if (z != NULL && z->point[d] < res->point[d]) 
       res = z; 
    return res; 
} 

Node *findMinRec(Node* root,int d,unsigned depth){
	// base cases
	if(root == NULL){
		return NULL;
	}

	unsigned cd  = depth % k;

	// compare with root with respect to cd 
	if(cd == d){
		if(root->left == NULL)
			return root;
		return findMinRec(root->left,d,depth+1);
	}
	return minNode(root,findMinRec(root->left,d,depth+1),findMinRec(root->right,d,depth+1),d);
}

Node *findMin(Node *root,int d){
	return findMinRec(root,d,0);
}

void copyPoint(double pt1[],double pt2[]){
	for(int i =0;i<k;i++){
		pt1[i] = pt2[i];
	}
	return;
}
Node *deleteNodeRec(Node *root,double point[],int depth){
	if(root == NULL){
		return NULL;
	}

	unsigned cd = depth % k;
	
	// if this is the point to be deleted
	if(arePointsSame(point,root->point)){

		if(root->right != NULL){
			// find min of root dim in right sub tree
			Node *min = findMin(root->right,cd);
			// copy the minimum root
			copyPoint(root->point,min->point);
			// recursively delete the minimum
			root->right = deleteNodeRec(root->right,min->point,depth+1);
		}
		else if(root->left !=NULL){
			Node* min = findMin(root->left,cd);
			copyPoint(root->point,min->point);
			root->right = deleteNodeRec(root->left,min->point,depth+1);
		}else{ // if node to be deleted is leaf node
			delete root;
			return NULL;
		}
		return root;
	}

	if(point[cd]<root->point[cd]){
		root->left = deleteNodeRec(root->left,point,depth+1);
	}else{
		root->right = deleteNodeRec(root->right,point,depth+1);
	}

	return root;
}
Node* deleteNode(Node* root,double point[])
{
	return deleteNodeRec(root,point,0);
}

double distance(double pt1[],double pt2[]){
	int sum_squares = 0;
	for(int i=0;i<k;i++){
		sum_squares+= ((pt1[i]-pt2[i])*(pt1[i]-pt2[i]));
	}
	return sqrt(sum_squares);
}

double guess = INT8_MAX;
Node* temp;
Node *nearestNodeRec(Node* root,double point[],unsigned depth){
	if(root==NULL){
		return NULL;
	}
	
	double curr_guess;
	curr_guess = distance(root->point,point);
	if(curr_guess < guess){
		guess = curr_guess;
		//cout<< "guess "<< guess<<endl;
		temp = root;
		//cout<<" temp print "<<temp->point[0]<<" "<<temp->point[1]<<" "<<temp->point[2]<<" "<<temp->point[3]<<" "<<temp->point[4]<<endl;
	}else{
		//cout<<" still temp print "<<temp->point[0]<<" "<<temp->point[1]<<" "<<temp->point[2]<<endl;
	}
	
	unsigned cd = depth % k;
	if(root->right!=NULL && abs(root->right->point[cd]-point[cd]) <= guess){ // if it's greater than guessed distance eleminate the entire subtree  
		//cout<<" right print "<<root->right->point[0]<<" "<<root->right->point[1]<<" "<<root->right->point[2]<<endl;
		root = nearestNodeRec(root->right,point,depth+1);

		//cout<<" temp right print "<<temp->point[0]<<" "<<temp->point[1]<<" "<<temp->point[2]<<endl;
	}else if(root->left!=NULL && abs(root->left->point[cd]-point[cd])<=guess){
		//cout<<" left print "<<root->left->point[0]<<" "<<root->left->point[1]<<" "<<root->left->point[2]<<endl;
		root = nearestNodeRec(root->left,point,depth+1);
		//cout<<" temp left print "<<temp->point[0]<<" "<<temp->point[1]<<" "<<temp->point[2]<<endl;
	}
	guess = INT8_MAX;
	return temp;

}
Node *nearestNode(Node* root,double point[])
{
	return nearestNodeRec(root,point,0);
} 

// int main() 
// { 
//     struct Node *root = NULL; 
//     int points[][k] = {{3, 6,2}, {17, 15,10}, {13, 15,7}, {6, 12,3}, 
//                        {9, 1, 1}, {2, 7,0}, {10, 19,6}}; 
  
//     int n = sizeof(points)/sizeof(points[0]); 
  
//     for (int i=0; i<n; i++) 
//        root = insert(root, points[i]); 
  
//     int point1[] = {10, 19, 6}; 
//     (search(root, point1))? cout << "Found\n": cout << "Not Found\n"; 
  
//     int point2[] = {12, 19,0}; 
//     (search(root, point2))? cout << "Found\n": cout << "Not Found\n"; 
//     cout << "Minimum of 0'th dimension is " << findMin(root, 0)->point[0]<<" "<<findMin(root, 0)->point[1]<<" "<<findMin(root, 0)->point[2]<< endl; 
//     cout << "Minimum of 1'th dimension is " << findMin(root, 1)->point[0]<<" "<<findMin(root, 1)->point[1]<<" "<<findMin(root, 1)->point[2] << endl; 
//     cout << "Minimum of 2'th dimension is " << findMin(root, 2)->point[0]<<" "<<findMin(root, 2)->point[1]<<" "<<findMin(root, 2)->point[2] << endl;
//     cout << "new node {0,50,20} "<<endl;
//     int point[] = {0,50,20};
//     root = insert(root,point);
//     cout << "Minimum of 0'th dimension is " << findMin(root, 0)->point[0]<<" "<<findMin(root, 0)->point[1]<<" "<<findMin(root, 0)->point[2]<< endl;
//     cout << "Root after deletion of (3, 6,2)\n"; 
//     int point4[] = {3,6,2};
//     root = deleteNode(root,point4);
//     cout << root->point[0] << ", " << root->point[1] << ", "<< root->point[2]<< endl; 
//     int point3[] = {-19,0,1};
//     cout<< "nearest point for {16,15,10} "<<nearestNode(root,point3)->point[0]<<" "<<nearestNode(root,point3)->point[1]<<" "<<nearestNode(root,point3)->point[2]<<endl;
  
//     return 0; 
// } 
