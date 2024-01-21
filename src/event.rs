trait HandlerFunctionBase<EventType> {
    fn call(&self, event: &EventType);
    fn equals(&self, func: usize) -> bool;
}

struct MemberFunctionHandler<T, EventType> {
    instance: *const T,
    member_function: fn(&T, &EventType),
}

impl<T, EventType> MemberFunctionHandler<T, EventType> {
    fn new(instance: &T, member_function: fn(&T, &EventType)) -> Self {
        MemberFunctionHandler {
            instance,
            member_function,
        }
    }
}

impl<T, EventType> HandlerFunctionBase<EventType> for MemberFunctionHandler<T, EventType> {
    fn call(&self, event: &EventType) {
        (self.member_function)(unsafe { &*self.instance }, event);
    }

    fn equals(&self, func: usize) -> bool {
        func == self.instance as usize ^ self.member_function as usize
    }
}

pub struct Dispatcher<EventType: 'static> {
    subscribed_functions: Vec<fn(&EventType)>,
    subscribed_member_functions: Vec<Box<dyn HandlerFunctionBase<EventType>>>,
}

impl<EventType> Dispatcher<EventType> {
    pub fn new() -> Self {
        Self {
            subscribed_functions: Vec::new(),
            subscribed_member_functions: Vec::new(),
        }
    }

    pub fn post(&self, event: EventType) {
        for func in &self.subscribed_functions {
            func(&event);
        }
        for func in &self.subscribed_member_functions {
            func.call(&event);
        }
    }

    pub fn subscribe<T: 'static>(&mut self, instance: &T, member_function: fn(&T, &EventType)) {
        let handler = Box::new(MemberFunctionHandler::new(instance, member_function));
        self.subscribed_member_functions.push(handler);
    }

    pub fn subscribe_func(&mut self, function: fn(&EventType)) {
        self.subscribed_functions.push(function);
    }

    pub fn unsubscribe_func(&mut self, function: fn(&EventType)) {
        self.subscribed_functions.retain(|&f| f != function);
    }

    pub fn unsubscribe<T: 'static>(&mut self, instance: &T, member_function: fn(&T, &EventType)) {
        let hash = instance as *const T as usize ^ member_function as usize;
        self.subscribed_member_functions.retain(|f| !f.equals(hash));
    }
}
