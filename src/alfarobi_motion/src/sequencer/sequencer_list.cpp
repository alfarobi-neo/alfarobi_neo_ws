#include "sequencer/sequencer_list.h"

void SequenceList::deleteSequence(std::string name)
{
    Sequence *temp1 = head, *temp2 = NULL;
    int ListLen = 0;
    int nodeOffset = 0;
  
    if (head == NULL) {
        ROS_WARN("LIST EMPTY");
        return;
    }
  
    // Find length of the linked-list.
    while (temp1 != NULL) {
        temp1 = temp1->next;
        ListLen++;
        if(temp1->name_ != name) {
            nodeOffset++;
        }
    }
  
    // Check if the position to be
    // deleted is greater than the length
    // of the linked list.
    if (ListLen < nodeOffset) {
        ROS_WARN("Index out of range");
        return;
    }
  
    // Declare temp1
    temp1 = head;
  
    // Deleting the head.
    if (nodeOffset == 1) {
  
        // Update head
        head = head->next;
        delete temp1;
        return;
    }
  
    // Traverse the list to
    // find the node to be deleted.
    while (nodeOffset-- > 1) {
  
        // Update temp2
        temp2 = temp1;
  
        // Update temp1
        temp1 = temp1->next;
    }
  
    // Change the next pointer
    // of the previous node.
    temp2->next = temp1->next;
  
    // Delete the node
    delete temp1;
}
  
// Function to insert a new node.
void SequenceList::insertSequence(alfarobi::joint_value *data, std::string name)
{
    // Create the new Node.
    Sequence* newNode = new Sequence(data, name);
  
    // Assign to head
    if (head == NULL) {
        head = newNode;
        return;
    }
  
    // Traverse till end of list
    Sequence* temp = head;
    while (temp->next != NULL) {
  
        // Update temp
        temp = temp->next;
    }
  
    // Insert at the last.
    temp->next = newNode;
}

void SequenceList::printList() {
    Sequence *temp = new Sequence();
    temp = head;
    // while(temp->next != NULL) {
    //     // std::cout<<temp->joint_->val[0];
    //     temp = temp->next;
    // }
    if(temp != nullptr) {
        std::cout<<"ADAAAA";
    } else {
        std::cout<<"KOSONGG GBLK";
    }
}
void Sequence::insertVal(double nval, int index) {
    joint_->val[index] = nval;
}
void Sequence::insertName(std::string n) {
    name_ = n;
}
void Sequence::insertTargetTime(double tt) {
    joint_->target_time = tt;
}
void Sequence::insertPauseTime(double pt) {
    joint_->pause_time = pt;
}