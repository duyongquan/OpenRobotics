#include "behaviortree_cpp_v3/bt_factory.h"

#include "dummy_nodes.h"
#include "movebase_node.h"

using namespace BT;

/** This tutorial will teach you how basic input/output ports work.
 *
 * Ports are a mechanism to exchange information between Nodes using
 * a key/value storage called "Blackboard".
 * The type and number of ports of a Node is statically defined.
 *
 * Input Ports are like "argument" of a functions.
 * Output ports are, conceptually, like "return values".
 *
 * In this example, a Sequence of 5 Actions is executed:
 *
 *   - Actions 1 and 4 read the input "message" from a static string.
 *
 *   - Actions 3 and 5 read the input "message" from an entry in the
 *     blackboard called "the_answer".
 *
 *   - Action 2 writes something into the entry of the blackboard
 *     called "the_answer".
*/

// clang-format off
static const char* xml_text = R"(

 <root main_tree_to_execute = "MainTree" >

     <BehaviorTree ID="MainTree">
        <Sequence name="root">
            <SaySomething     message="hello" />
            <SaySomething2    message="this works too" />
            <ThinkWhatToSay   text="{the_answer}"/>
            <SaySomething2    message="{the_answer}" />
        </Sequence>
     </BehaviorTree>

 </root>
 )";
// clang-format on

class ThinkWhatToSay : public BT::SyncActionNode
{
public:
  ThinkWhatToSay(const std::string& name, const BT::NodeConfiguration& config) :
    BT::SyncActionNode(name, config)
  {}

  // This Action simply write a value in the port "text"
  BT::NodeStatus tick() override
  {
    // 在名为"text" port 写入值 "The answer is 42"
    setOutput("text", "The answer is 42");

    // getInput<std::string>("message");
    return BT::NodeStatus::SUCCESS;
  }

  // 表示 该BT::Node中 有一个 要写入值的port， 该port的名称是"text"
   // 在一个BT::Node中 声明一个port的方式
  static BT::PortsList providedPorts()
  {
    // 要写入值的port 的名字叫做 "text"
    return {BT::OutputPort<std::string>("text")};
  }
};

int main()
{
  using namespace DummyNodes;

  BehaviorTreeFactory factory;

  // The class SaySomething has a method called providedPorts() that define the INPUTS.
  // In this case, it requires an input called "message"
  factory.registerNodeType<SaySomething>("SaySomething");

  // Similarly to SaySomething, ThinkWhatToSay has an OUTPUT port called "text"
  // Both these ports are std::string, therefore they can connect to each other

  // void BT::BehaviorTreeFactory::registerNodeType(const std::string& ID)
  // registerNodeType is the method to use to register your custom TreeNode
  // It accepts only classed derived from either ActionNodeBase, DecoratorNode, ControlNode, or ConditionNode
  factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay");

  // SimpleActionNodes can not define their own method providedPorts(), therefore
  // we have to pass the PortsList explicitly if we want the Action to use getInput()
  // or setOutput();
  // 要读取值的port 的名字叫做 "message"
  PortsList say_something_ports = {InputPort<std::string>("message")};

  // void BT::BehaviorTreeFactory::registerSimpleAction(const std::string&  ID
  //                                                    const SimpleConditionNode::TickFunctor& tick_functor  
  //                                                    PortList        ports={}
  // )
  factory.registerSimpleAction("SaySomething2", SaySomethingSimple, say_something_ports);

  /* An INPUT can be either a string, for instance:
     *
     *     <SaySomething message="hello" />
     *
     * or contain a "pointer" to a type erased entry in the Blackboard,
     * using this syntax: {name_of_entry}. Example:
     *
     *     <SaySomething message="{the_answer}" />
     */
  
  // 在BT中使用 Port 使BT::Node之间的数据通信 ，一定要xml中 的 写和读 的port的 key相同
  // "{the_answer}"

  // <ThinkWhatToSay   text="{the_answer}"/>, 名为“text”的port的 key为 "{the_answer}"
  // <SaySomething2    message="{the_answer}" />, 名为“message”的port的 key为 "{the_answer}"
  auto tree = factory.createTreeFromText(xml_text);
  tree.tickRootWhileRunning();

  /*  Expected output:
     *
        Robot says: hello
        Robot says: this works too
        Robot says: The answer is 42
    *
    * The way we "connect" output ports to input ports is to "point" to the same
    * Blackboard entry.
    *
    * This means that ThinkSomething will write into the entry with key "the_answer";
    * SaySomething and SaySomething will read the message from the same entry.
    *
    */
  return 0;
}