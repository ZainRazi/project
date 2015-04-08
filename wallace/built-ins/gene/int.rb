# encoding: utf-8
# import('numeric')

# define_type :int do
#   extends(:gene, :numeric)
#   #ruby_type(ruby(:integer))
#   composer { s(:gene, :int) }
# end

# define_specification :int do
#   extends(:gene, :numeric, [t: ruby(:int)])
#   implement_method :sample do
#     rng.integer(min, max, inclusive)
#   end
# end
