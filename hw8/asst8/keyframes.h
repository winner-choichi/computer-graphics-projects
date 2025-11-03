#ifndef KEYFRAMES_H
#define KEYFRAMES_H

#include <memory>
#include <vector>
#include <list>
#include <algorithm>
#include <fstream>
#include <sstream>

#include "rigtform.h"
#include "sgutils.h"
#include "scenegraph.h"

struct Paster : public SgNodeVisitor
{
private:
    std::vector<RigTForm> frame_;
    int paste_node_counter;

public:
    // root부터가 아니라 임의의 노드부터 씬에 붙여넣기를 하는 확장을 대비한 offset적용. 지금은 무조건 0이어야 함
    Paster(std::vector<RigTForm> &frame, int offset = 0) : frame_(frame), paste_node_counter(offset) {};

    virtual bool visit(SgTransformNode &node)
    {
        std::shared_ptr<SgRbtNode> node_ = std::dynamic_pointer_cast<SgRbtNode>(node.shared_from_this());
        if (node_)
        {
            node_->setRbt(frame_[paste_node_counter]);
            paste_node_counter += 1;
        }
        return true;
    }
};

class Frame
{
public:
    Frame() {};
    Frame(std::vector<RigTForm> f) : frame_(f) {};

    void capture_scene(std::shared_ptr<SgNode> root)
    {
        std::vector<std::shared_ptr<SgRbtNode>> dump;
        dumpSgRbtNodes(root, dump);
        for (const auto &p : dump)
        {
            frame_.push_back(p->getRbt());
        }
    }

    bool paste_to_scene(std::shared_ptr<SgNode> root)
    {
        Paster paster(frame_);
        if (root->accept(paster))
        {
            return true;
        }
        else
        {
            std::cout << "paste error" << std::endl;
            return false;
        }
    }

    std::string get_frame()
    {
        std::string s;
        for (const auto &node : frame_)
        {
            const RigTForm rbt_ = node;
            const Cvec3 t = rbt_.getTranslation();
            const Quat q = rbt_.getRotation();
            // s += "rbt ";
            s += std::to_string(t[0]) + " " + std::to_string(t[1]) + " " + std::to_string(t[2]) + " ";
            s += std::to_string(q[0]) + " " + std::to_string(q[1]) + " " + std::to_string(q[2]) + " " + std::to_string(q[3]) + "\n";
        }
        return s;
    }

    std::vector<RigTForm> get_rbts() const
    {
        return frame_;
    }

    void set_frame(std::vector<RigTForm> frame)
    {
        frame_ = frame;
    }

private:
    std::vector<RigTForm> frame_;
};

struct Script
{
    using list_iter = std::list<std::shared_ptr<Frame>>::iterator;

private:
    std::list<std::shared_ptr<Frame>> frames;
    list_iter curr_frame;
    std::shared_ptr<SgNode> root_;

public:
    Script() {};

    Script(std::shared_ptr<SgNode> root)
    {
        curr_frame = frames.end();
        root_ = root;
    };

    // n key
    void add_frame()
    {
        std::shared_ptr<Frame> f = std::make_shared<Frame>(Frame());
        f->capture_scene(root_);
        if (curr_frame == frames.end())
        {
            frames.push_back(f);
            curr_frame = std::prev(frames.end());
        }
        else
        {
            ++curr_frame;
            curr_frame = frames.insert(curr_frame, f);
        }
        std::cout << "Create new frame [" << std::distance(frames.begin(), curr_frame) << "]" << std::endl;
    }

    // u key
    void update_frame()
    {
        if (curr_frame == frames.end())
        {
            this->add_frame();
        }
        else
        {
            std::shared_ptr<Frame> f = std::make_shared<Frame>(Frame());
            f->capture_scene(root_);
            curr_frame = frames.erase(curr_frame);
            curr_frame = frames.insert(curr_frame, f);
        }
        std::cout << "Copying scene graph to current frame [" << std::distance(frames.begin(), curr_frame) << "]" << std::endl;
    }

    // space key
    void paste_curr_frame()
    {
        if (curr_frame == frames.end())
        {
            std::cout << "No key frame defined" << std::endl;
        }
        else
        {
            std::cout << "Loading current key frame [" << std::distance(frames.begin(), curr_frame) << "] to scene graph" << std::endl;
            (*curr_frame)->paste_to_scene(root_);
        }
    }

    // > key
    void move_next_frame()
    {
        if (curr_frame != frames.end() && curr_frame != std::prev(frames.end()))
        {
            ++curr_frame;
            std::cout << "Stepped forward to frame [" << std::distance(frames.begin(), curr_frame) << "]" << std::endl;
            (*curr_frame)->paste_to_scene(root_);
        }
    }

    // < key
    void move_prev_frame()
    {
        if (curr_frame != frames.begin())
        {
            --curr_frame;
            std::cout << "Stepped backward to frame [" << std::distance(frames.begin(), curr_frame) << "]" << std::endl;
            (*curr_frame)->paste_to_scene(root_);
        }
    }

    // d key
    void delete_frame()
    {
        if (curr_frame != frames.end())
        {
            std::cout << "Deleting current frame [" << std::distance(frames.begin(), curr_frame) << "]" << std::endl;
            curr_frame = frames.erase(curr_frame);
            int d = std::distance(frames.begin(), curr_frame);
            if (curr_frame == frames.end())
            {
                std::cout << "No frames defined" << std::endl;
            }
            else if (d == 0)
            {
                std::cout << "Now at frame [" << d << "]" << std::endl;
                (*curr_frame)->paste_to_scene(root_);
            }
            else
            {
                std::cout << "Now at frame [" << d - 1 << "]" << std::endl;
                --curr_frame;
                (*curr_frame)->paste_to_scene(root_);
            }
        }
    }

    // w key
    void get_script(const std::string &filename)
    {
        std::ofstream out(filename);
        if (!out.is_open())
        {
            std::cout << "Faild to open " << filename << std::endl;
            return;
        }
        std::cout << "Writing animation to " << filename << std::endl;
        int i = 0;
        for (const auto &f : frames)
        {
            out << "frame [" << i << "]\n"
                << f->get_frame();
            ++i;
        }
        out.close();
    }

    // i key
    void set_script(const std::string &filename)
    {
        std::ifstream in(filename);
        if (!in.is_open())
        {
            std::cout << "Faild to open " << filename << std::endl;
            return;
        }

        std::string line;
        std::vector<RigTForm> rbts;
        frames.clear();
        list_iter iter = frames.begin();

        int frame_counter = 0;
        while (std::getline(in, line))
        {
            std::shared_ptr<Frame> f = std::make_shared<Frame>();
            RigTForm rbt;

            std::string token;
            std::istringstream iss(line);
            if (iss >> token && token == "frame")
            {
                if (rbts.size() != 0)
                {
                    f->set_frame(rbts);
                    iter = frames.insert(iter, f);
                    ++iter;
                    ++frame_counter;
                }
                rbts.clear();
                continue;
            }

            std::istringstream iss2(line);
            double x, y, z, qw, qx, qy, qz;
            if (iss2 >> x >> y >> z >> qw >> qx >> qy >> qz)
            {
                rbt.setTranslation(Cvec3(x, y, z));
                rbt.setRotation(Quat(qw, qx, qy, qz));
                rbts.push_back(rbt);
            }
        }
        if (!rbts.empty()) // Add final frame if remaing
        {
            std::shared_ptr<Frame> f = std::make_shared<Frame>();
            f->set_frame(rbts);
            frames.insert(iter, f);
            ++frame_counter;
        }

        std::cout << frame_counter << " frames read." << std::endl;
        curr_frame = frames.begin();
        (*curr_frame)->paste_to_scene(root_);
        std::cout << "Now at frame [0]" << std::endl;
    }

    int get_script_size()
    {
        return frames.size();
    }

    Frame get_frame_at(int i) const
    {
        if (i < 0 || i >= frames.size())
        {
            std::cout << "Index out of range" << std::endl;
            return Frame();
        }

        std::list<std::shared_ptr<Frame>>::const_iterator iter = frames.begin();
        std::advance(iter, i);
        return **iter;
    }

    void set_currframe_to_n_display(int i)
    {
        list_iter iter = frames.begin();
        std::advance(iter, i);
        curr_frame = iter;
        (*curr_frame)->paste_to_scene(root_);
    }
};

#endif