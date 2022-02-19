//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) by Christian Gloor
//

#ifndef _ped_tree_h_
#define _ped_tree_h_ 1

//disable warnings on 255 char debug symbols
#pragma warning (disable : 4786)
//disable warnings on extern before template instantiation
#pragma warning (disable : 4231)

#ifdef _WIN32
#ifdef _DLL
#    define LIBEXPORT __declspec(dllexport)
#    define EXPIMP_TEMPLATE
#else
#    define LIBEXPORT __declspec(dllimport)
#    define EXPIMP_TEMPLATE extern
#endif
#else
#    define LIBEXPORT
#    define EXPIMP_TEMPLATE
#endif

#include <set>

using namespace std;

namespace Ped {
    class Tagent;
    class Tscene;
}
   
EXPIMP_TEMPLATE template class LIBEXPORT std::set<const Ped::Tagent*>;

namespace Ped {
    class LIBEXPORT Ttree {
        friend class Tscene;

    public:
        Ttree(Ped::Tscene *scene, int depth, double x, double y, double w, double h);
        virtual ~Ttree();

        virtual void clear();

        virtual void addAgent(const Ped::Tagent *a);
        virtual void moveAgent(const Ped::Tagent *a);
        virtual bool removeAgent(const Ped::Tagent *a);

        virtual set<const Ped::Tagent*> getAgents() const;
        virtual void getAgents(list<const Ped::Tagent*>& outputList) const;

        virtual bool intersects(double px, double py, double pr) const;

        double getx() const { return x; };
        double gety() const { return y; };
        double getw() const { return w; };
        double geth() const { return h; };

        double getdepth() const { return depth; };

    protected:
        virtual int cut();
        virtual void addChildren();
        Ttree* getChildByPosition(double x, double y);

        bool isleaf;
        double x;
        double y;
        double w;
        double h;
        int depth;

        Ttree *tree1;
        Ttree *tree2;
        Ttree *tree3;
        Ttree *tree4;

        Ped::Tscene *scene;

    private:
       set<const Ped::Tagent*> agents;	// set and not vector, since we need to delete elements from the middle very often
										// set and not list, since deletion is based on pointer (search O(log n) instead of O(n)).

	
	};
}

#endif
