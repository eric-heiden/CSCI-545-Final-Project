#ifndef STEPSEQUENCE_H
#define STEPSEQUENCE_H
#include <vector>

typedef void (*AssignFunctionPtr)(void);
typedef void (*MoveFunctionPtr)(double);

class Runnable
{
public:
    virtual ~Runnable()
    {}

    virtual void execute() = 0;
    virtual bool hasFinished() const = 0;

    virtual void initialize()
    {
        SIM_LOG("Executing " << name());
    }

    virtual std::string name() const
    {
        return "Runnable";
    }
};

/**
 * @brief Abstracts a step which first assigns a target and makes the required movements.
 */
class Step : public Runnable
{
public:
    Step(std::string name, AssignFunctionPtr assign, MoveFunctionPtr move, double tau, double delta_t)
        : _name(name), _tau(tau), _time(0), _delta_t(delta_t), _assigned(false)
    {
        _assign = assign;
        _move = move;
    }

    bool hasFinished() const
    {
        return _time + _delta_t/2 >= _tau;
    }

    void initialize()
    {
        Runnable::initialize();
        _time = 0;
        _assigned = false;
    }

    void execute()
    {
        if (!_assigned)
        {
            _assign();
            _assigned = true;
        }
        else
        {
            _move(_tau-_time);
            _time += _delta_t;
        }
    }

    std::string name() const
    {
        return _name;
    }

    AssignFunctionPtr _assign;
    MoveFunctionPtr _move;

private:
    std::string _name;
    double _tau;
    double _time;
    double _delta_t;
    bool _assigned;
};

/**
 * @brief Executes a sequence of steps.
 */
class StepSequence : public Runnable
{
public:
    bool cycle;

    StepSequence(bool cycle = false) : cycle(cycle)
    {
        _active = _steps.begin();
    }

    virtual ~StepSequence()
    {
        for (_active = _steps.begin(); _active != _steps.end(); ++_active)
            delete *_active;
    }

    void add(Runnable *step)
    {
        _steps.push_back(step);
    }

    void initialize()
    {
        _active = _steps.begin();
        if (_active != _steps.end())
            (*_active)->initialize();
    }

    void execute()
    {
        if (_active == _steps.end())
            return;

        if ((*_active)->hasFinished())
        {
            ++_active;
            if (_active == _steps.end())
            {
                if (cycle)
                {
                    SIM_LOG("Completed cycle");
                    initialize();
                }
                else
                    SIM_LOG("Finished execution");

                return;
            }
            else
            {
                (*_active)->initialize();
            }
        }

        (*_active)->execute();
    }

    bool hasFinished() const
    {
        return !cycle && _active == _steps.end();
    }

    std::string name() const
    {
        return "Step Sequence";
    }

private:
    std::vector<Runnable*> _steps;
    std::vector<Runnable*>::iterator _active;
};

#endif // STEPSEQUENCE_H
